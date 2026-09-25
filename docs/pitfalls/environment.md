# 环境与踩坑记录（本机）

在本机（Linux + Optimus 双显卡：Intel UHD 核显 + NVIDIA MX350）搭建和运行本仓库环境时踩到的坑。 这里既是"踩坑备查"，也是仓库里各项配置的**依据**——改配置前先读对应小节。

相关文档：MuJoCo 本体知识点与坑点见 [`../learn/mujoco.md`](../learn/mujoco.md)（第 7 节讲 `MUJOCO_GL` 与渲染开销，[§7.6](../learn/mujoco.md#76-这些结论驱动了哪些配置决策) 是"结论 → 配置"的反向索引）； 图形栈背景见 [`../learn/graphics-stack.md`](../learn/graphics-stack.md)；编辑器提示的完整方案见 [`../learn/cmake-intellisense.md`](../learn/cmake-intellisense.md)；项目约定见 [`../conventions.md`](../conventions.md)；文档索引见 [`../../README.md`](../../README.md)。

## 工具链选择： Pixi = Conda + uv

好处：**Python、MuJoCo（C 库 + Python 绑定）与 C++ 工具链都装在同一个 pixi 环境里**，不用系统 apt 包，也不用单独的 venv。

- **跨语言共用同一份 MuJoCo**：任务 4 要写 C++，而 C++ 需要的是 `libmujoco`（头文件 + 库 + CMake 配置）；conda-forge 的 `mujoco` 包同时提供 C 库与 Python 绑定，两边**就是同一个 3.12.0**，行为一致、数字可比（C++/Python 对照正是任务 4 的验收方式）。pip/venv 只能拿到 Python 轮子，拿不到配套头文件与 CMake 配置。
- **一次装齐 C++ 工具链**：`cxx-compiler` / `cmake` / `ninja` 都在 conda-forge，与 `libmujoco` 出自同一条工具链，避免“系统 gcc 编、conda 库链接”的 ABI / libstdc++ 错配。
- **可复现**：`pixi.lock` 锁死确切版本与哈希，别人 clone 后 `pixi install` 得到同一个环境，“在我机器上能跑”这件事有据可查。
- **不污染系统**：一切装在仓库内的 `.pixi/`（已 gitignore），不需要 sudo，删掉目录即回滚；系统只负责显卡驱动这类必须由系统管的东西。
- **能承载“必须提前生效”的环境变量**：`MUJOCO_GL` 必须在 `import mujoco` 之前生效（见下面“图形后端”一节），`[activation.env]` 正好解决这件事。
- **两种包源都可用**：conda 侧走 conda-forge（实测直连 conda.anaconda.org 最快，见下一小节），PyPI 侧由内置的 uv 负责，并可用 `extra-index-urls` 配国内回退源。
- 代价：多一层工具（要习惯用 `pixi run` 而不是直接 `python`），且 conda-forge 的 `mujoco` 版本略落后于 PyPI（当前 3.12.0 vs 3.14.0）——对本项目不构成问题。

## 2026-09-24 pixi / conda / PyPI 镜像

本工作区用 `pixi` 统一管理 Python 与 MuJoCo（`pixi.toml` 在仓库根目录）。初始化时遇到下面几个坑，均已处理，记录备查。

**1）清华 TUNA 镜像对本机整体 403（conda 与 pypi 都是）**

```bash
curl -s -o /dev/null -w '%{http_code}\n' \
  https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/linux-64/repodata.json   # 403
curl -s -o /dev/null -w '%{http_code}\n' \
  https://pypi.tuna.tsinghua.edu.cn/simple/mujoco/                                          # 403
```

现象：`pixi search` / `pixi install` 直接失败（`HTTP status client error (403 Forbidden)`）； 只有 `pixi search --no-config ...` 才能绕过配置拿到数据。

- 配置位置是 `~/.pixi/config.toml`（**不是** `~/.config/pixi/config.toml`），原内容把 `conda-forge` / `pytorch` 映射到 TUNA，并把 pypi `index-url` 也指向 TUNA。
- 处理：**删除 `[mirrors]` 段**，pypi `index-url` 改成 USTC。原因见第 2 条。

**2）国内镜像都不支持 sharded repodata，走镜像反而更慢**

conda 的完整 `repodata.json` 是几十 MB；conda.anaconda.org 额外提供 `repodata_shards.msgpack.zst`（linux-64 约 **570 KB**），pixi 会优先用它。实测：

| 源 | `repodata_shards.msgpack.zst` |
|---|---|
| conda.anaconda.org（官方） | **200**，570 KB / 2.3 s |
| USTC | 302（跳 NJU） |
| NJU / aliyun / TUNA | 404 |

所以结论是反直觉的：**conda 侧不用镜像，直连 conda.anaconda.org 最快**。这也顺便解释了 第 1 条里为什么「删掉 mirrors」是正确解法。

**3）PyPI 上游本身达不到 1 MiB/s，所以仍然需要镜像**

同一文件（`mujoco-3.14.0-cp312-...whl`）8~20 s 限时下载实测：

| 源 | 实测速度 |
|---|---|
| files.pythonhosted.org（上游） | 4～21 KB/s |
| USTC | 31～356 KB/s（最快） |
| NJU | 157 KB/s |
| aliyun | 索引 200，但文件路径 404（布局不同） |
| pypi.org | 索引可访问，但 20 s 超时 |

结论：**没有任何一端达到 1 MiB/s**，因此保留镜像；`pixi.toml` 用多索引回退链：

```toml
[pypi-options]
extra-index-urls = [
    "https://mirrors.ustc.edu.cn/pypi/simple",
    "https://mirrors.aliyun.com/pypi/simple",
    "https://mirror.nju.edu.cn/pypi/web/simple",
]
```

- pixi 底层是 uv，**采用 first-index 策略**（不是 pip 的「所有索引一起选最优版本」）： 按顺序取**第一个命中该包**的索引，且 `extra-index-urls` **优先于** `index-url`。 上面 `index-url` 留空 ⇒ 默认 `https://pypi.org/simple` 作为最后兜底。
- 因此 `extra-index-urls` 的顺序 = 优先级顺序，**第一个应该写最快的源**（此处 USTC）。
- 注意 pip/uv 需要的是 PEP 503 的 `.../simple` 路径；`https://mirrors.ustc.edu.cn/pypi/` 会 301，不能直接当 index 用。

**4）其它小坑**

| 问题 | 说明 |
|---|---|
| `pixi add --dry-run` | pixi 0.77.0 **没有**这个参数 |
| `pixi --no-config search ...` | 报 `unexpected argument`；`--no-config` 必须放在**子命令之后**：`pixi search --no-config ...` |
| conda-forge 的 `mujoco` 版本落后 | conda-forge 最高 **3.12.0**，PyPI 已到 **3.14.0**；本项目选 conda-forge 以统一 C++/Python |
| `raw.githubusercontent.com` | 一度超时（curl exit 28）；稍后恢复。URDF 来源用 sha256 校验确认 |
| pixi 识别到 `__cuda=13.0` | 来自驱动 580；但本机 MX350 是 sm_61，**不支持 CUDA 13**，后续装 CUDA 相关包时要注意 |

**测速/验源可复用的命令：**

```bash
# conda 镜像是否支持 sharded repodata（200 才值得用）
curl -s -o /dev/null -w '%{http_code}\n' \
  "$MIRROR/conda-forge/linux-64/repodata_shards.msgpack.zst"

# 真实吞吐（B/s；>= 1048576 才算 1 MiB/s）
curl -s -o /dev/null -m 10 -w '%{http_code} %{speed_download}\n' "$FILE_URL"
```

## 2026-09-24 URDF → MJCF 转换与 git 索引

详见 [`../../@20260923_mujoco/README.md`](../../@20260923_mujoco/README.md)，这里只记最容易再踩的几条：

1. **MuJoCo 自带转换会把根 link 并进 `worldbody`**。`mj_saveLastXML()` / `MjSpec.from_file().to_xml()`读 URDF 得到的结果里，`trunk` 不再是 body ⇒ **没有 `<freejoint/>`，而且基座质量惯量整块丢失**（本模型 13.2472 → 7.47 kg，差值 5.7772 kg 正好是 trunk+imu_link+d435_link）。它还只导 URDF 的 `<collision>`，visual mesh 全部丢弃。
2. **urdf.enkeebot.com 的「Include Skeleton」版没有碰撞体**：19 个 geom 全是`contype=0 conaffinity=0`，用它做“趴在地上”会直接穿过地面。正确组合是**Floating Base 开 + Actuator Type = Torque + Include Skeleton 关**。
3. **同一份 mesh 被复制了 3 份**（原始 URDF、网站导出、旧导出），每份 34 MB 且 md5 完全相同。处理：模型用 `meshdir` 指向 `assets/urdf/meshes` 这一份，其余在 `.gitignore` 里排除。
4. **`.gitignore` 对已经 `git add` 过的文件无效**。重复的 meshes 与已删除的旧 skeleton 仍然留在索引里，必须 `git rm -r --cached <path>` 才能真正排除（磁盘文件不受影响）。
5. **导出模型的默认位形穿模**。urdf.enkeebot.com 把根 body 放在原点，而零位形下脚底在基座下方约 0.58 m ⇒ 任何“建完 `MjData` 直接 `mj_step`”的脚本都会看到求解器把狗弹到空中（场景里的 `<keyframe>` 不会被自动加载）。两条修法：把模型基座默认高度抬到“脚底刚好触地”（推荐，模型自洽），或在脚本里`mj_resetDataKeyframe(model, data, 0)` / 显式设 `data.qpos[2]`。
6. **`<include>` 之后 `meshdir` 是相对顶层文件解析的**。scene 与模型分目录时，模型里的`meshdir="meshes/"` 会去 scene 所在目录找 `meshes/`，而同一模型单独加载却正常。做法：每个会用到它的目录各放一个 `meshes` 目录软链接指向唯一一份实际在存储的 mesh （git 也能直接存软链接）；当然，直接在xml中改引用路径也可以。

> MuJoCo 知识点（`geom` 的 `type` 取值、`friction` / `condim` 语法与默认值、接触参数速查） 见 [`../learn/mujoco.md`](../learn/mujoco.md)。

## 2026-09-25 图形后端 / GPU / 渲染性能（本机实测）

> 数据与复现方式见 [`../learn/mujoco.md`](../learn/mujoco.md) 第 6、7 节， 图形栈背景见 [`../learn/graphics-stack.md`](../learn/graphics-stack.md)。

- 本机是 **Optimus 双显卡**（Intel UHD 核显负责显示 + NVIDIA MX350 2 GB 独显），实测 **`MUJOCO_GL` 会决定用哪块卡**：
  * `egl` → 离屏 context 落在 **NVIDIA MX350**（GL 4.6，max texture 32768）；
  * `glfw` → 走“显示的 GL”，即 **Intel 核显**（Mesa 23.2.1，max texture 16384）。

  所以两侧的性能差异**首先是 GPU 差异**，不是后端本身的开销。
- 渲染成本（960×540，同一套代码）：MX350 出一帧 ~5.5 ms、整条录像链路 ~7.1 ms； 核显则分别是 ~21 ms / ~22 ms（50 fps 录像要 ~111% CPU，跟不上）。
- 仓库把 `MUJOCO_GL=egl` 写进 `pixi.toml` 的 `[activation.env]`，对所有脚本生效； 临时换后端要 `pixi run env MUJOCO_GL=glfw python …`（命令行前缀会被激活环境覆盖）。
- 本机显卡算力（Pascal sm_61 + 驱动 580）**不够跑 MJX / MJWarp**，所以只用 CPU 仿真。
- 本机特有的小现象：开窗口的进程退出时偶发 `segmentation fault` 或 `GLFWError: EGL: Failed to clear current context`（MP4 在崩溃前已写完，产物不受影响）； Wayland/XWayland 会把 GLFW 的窗口位置警告打到 stderr。
- 这些结论**驱动了哪些配置**（反向索引）：见 [`../learn/mujoco.md` 第 7.6 节](../learn/mujoco.md#76-这些结论驱动了哪些配置决策)。

## 2026-09-25 C++ 工具链（pixi 提供）与编辑器提示

> 用途与任务背景见 [`../../@20260923_mujoco/README.md`](../../@20260923_mujoco/README.md) 的「C++ 程序」一节。

- **用 pixi 的编译器，不用系统 gcc**：conda-forge 的 `libmujoco` 是 conda 工具链编出来的，混搭系统 gcc 容易踩 ABI / libstdc++ 版本问题，所以 `cxx-compiler` / `cmake` / `ninja` 都装进同一个 env（本机实测 gcc 15.3.0 / cmake 4.4.3 / ninja 1.13.2）。
- conda 的 `mujoco` 包**已经带齐 C++ 需要的东西**：`include/mujoco/*.h`、`libmujoco.so`、`lib/cmake/mujoco/mujocoConfig.cmake`（目标 `mujoco::mujoco` 与 `mujoco::libmujoco_simulate`），所以既不用自带 MuJoCo 源码，也不用宇树 readme 里那套“下载官方包解压到 `~/.mujoco` 再 `ln -s`”。
- 配置时把 `-DCMAKE_PREFIX_PATH="$CONDA_PREFIX"` 传进去即可，`find_package(mujoco)` 就能找到上面的 CMake 配置。
- **换 C++ 不会更快**：`mj_step` 两边调的是同一份 C 库，本机实测 C++ 循环 0.0394 ms/步、Python 0.04 ms/步；渲染依旧是 5.5 ms/帧（由 GPU 决定）。
- **编辑器（clangd）要配 `--query-driver`**：CMake 把 `$CONDA_PREFIX/include` 当作“隐式包含目录”而**不写进** `compile_commands.json`，clangd 于是找不到 `mujoco/mujoco.h`、也拿不到 conda 的 libstdc++ 头，满屏标红（一次自检可报出 21 条错误）。让 clangd 去问 pixi 的编译器即可消除：`--query-driver=**/.pixi/envs/*/bin/*`（glob 必须 `**/` 开头，实测不带就匹配不上）。排查工具：`clangd --check=<file>`，它打印的就是编辑器同源的诊断。
- **这个参数要写在「工作区文件」或用户设置里，不能写在文件夹级 `.vscode/settings.json`**：clangd 扩展把 `clangd.arguments` 声明为 **window scope**，而多根工作区下 window 级设置只认工作区文件 / 用户设置，放文件夹里不生效（现象：clangd 进程参数是空的，仍然标红）。本项目放在同级的 `RoboCon.code-workspace`（该文件不入库）。
- 改完要重启语言服务器（命令面板 → `clangd: Restart language server`），否则跑的还是旧进程；想确认可以直接看进程参数：`ps -eo args | grep clangd`。
- clangd 把索引缓存写在 `<project>/.cache/clangd/`（它把含 `compile_commands.json` 的上级目录当作 project），已加进 `.gitignore`。
