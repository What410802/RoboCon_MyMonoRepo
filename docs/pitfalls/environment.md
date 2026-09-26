# 环境与踩坑记录（本机）

在本机（Linux + Optimus 双显卡：Intel UHD 核显 + NVIDIA MX350）搭建和运行本仓库环境时踩到的坑。 这里既是"踩坑备查"，也是仓库里各项配置的**依据**——改配置前先读对应小节。

相关文档：MuJoCo 本体知识点与坑点见 [`../learn/mujoco.md`](../learn/mujoco.md)（第 7 节讲 `MUJOCO_GL` 与渲染开销，[§7.6](../learn/mujoco.md#76-这些结论驱动了哪些配置决策) 是"结论 → 配置"的反向索引）； 图形栈背景见 [`../learn/graphics-stack.md`](../learn/graphics-stack.md)；编辑器提示的完整方案见 [`../learn/cmake-intellisense.md`](../learn/cmake-intellisense.md)；项目约定见 [`../conventions.md`](../conventions.md)；文档索引见 [`../../README.md`](../../README.md)。

## 环境怎么搭出来（从零复现）

**安装步骤（装 pixi → `pixi install` → 验证）在仓库根 [`README.md`](../../README.md) 的「快速开始」里，不在这里重复。** 本节只记“为什么 / 谁提供”的部分：

- `[dependencies]` 只声明了 5 项：`python=3.12.*`、`mujoco>=3.12.0,<4`、`cxx-compiler`、`cmake`、`ninja`；其余全是它们的依赖自动带进来的。**MuJoCo 的 C++ 头文件、库与 CMake 配置不是我们单独装的**，而是 `mujoco` 这个**元包**（它自己 0 个文件）依赖的 `libmujoco` 提供：`include/mujoco/*.h`、`include/simulate/*.h`、`lib/libmujoco.so`、`lib/libmujoco_simulate.so`、`lib/cmake/mujoco/`；`glfw`、`libgl-devel`、`mujoco-simulate`、`mujoco-samples` 也都是随它进来的。

```bash
# 各项依赖是谁装的（mujoco 是元包，libmujoco 才真正带头文件）
python3 - <<'PY'
import glob, json
for f in sorted(glob.glob('.pixi/envs/default/conda-meta/*mujoco*.json')):
    d = json.load(open(f))
    n = len([x for x in d['files']
             if x.startswith(('include/mujoco', 'lib/libmujoco', 'lib/cmake/mujoco'))])
    print(f"{d['name']:16s} {d['version']:8s} 相关文件 {n:3d} 个")
PY

```

搭环境时踩过的坑（清华镜像 403、镜像不支持 sharded repodata、PyPI 上游达不到 1 MiB/s 所以要保留回退链）见下面各节；任务目录只引用本文，不重复。

**能不能真的从零装全**（2026-09-26 实测）：把仓库 clone 到一个干净目录（新克隆里没有 `.pixi`），只跑 `pixi install`——2 s 就装完（包都在 pixi 本地缓存里，所以没走网络；换一台全新机器会重新下载，耗时见后面镜像一节），随后 `import mujoco` 得到 3.12.0、`include/mujoco/mujoco.h` 与 `lib/libmujoco.so` 都在，Python 侧 `scripts/agent_scripts/rest_check.py` 与 C++ 侧 `cpp_task2`（cmake + ninja 编译后运行）都直接跑通且结论一致（8 s 后末 1 s 漂移 4.440e-10 m、判“静止趴住”）。

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

1. **MuJoCo 自带转换会把根 link 并进 `worldbody`**：没有 `<freejoint/>`，基座的质量惯量也整块丢失（本模型 13.2472 → 7.47 kg），而且只导 `<collision>`、visual mesh 全丢。
2. **urdf.enkeebot.com 的「Include Skeleton」版没有碰撞体**（全部 geom `contype=0 conaffinity=0`），拿它做“趴在地上”会直接穿过地面。
   上面两条的症状、数字与选项组合见 [`../../@20260923_mujoco/docs/model.md`](../../@20260923_mujoco/docs/model.md)（本任务文档），这里只留结论。
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
- **离屏渲染的帧尺寸由模型的 `<visual><global offwidth/offheight>` 决定，不是请求尺寸，也不是窗口尺寸**（默认 640×480；本任务场景声明的是 1280×720）。以前 `cpp_task2/src/record.h` 没动这两个字段，于是离屏 buffer 一直是场景声明的 1280×720，再把 1280×720 的帧按请求的 960×540 喂给 ffmpeg，帧就错位（现象：写进 191 帧的 raw 流被 ffmpeg 读成 339 帧、时长 6.78 s）。现在 recorder 在 `mjr_makeContext` **之前**把 `offwidth/offheight` 设成输出尺寸（Python 侧 `VideoRecorder` 一直这么做），实测 960×540 / 1920×1080 / 3840×2160 三档的视口与输出尺寸逐项一致、ffprobe 实测分辨率也对得上，`vflip` 后面不再需要 `scale`；只有驱动把视口夹小时才回退到"按实际视口读 + scale 到输出"（`mjr_maxViewport` 仍是每帧字节数的唯一依据）。
- **录像链路的开销量级**（`cpp_slope --mode record --pitch 15 --seconds 5`，251 帧 / 5 仿真秒，`egl`→MX350）：960×540 → 9.8–13.3 s（39–53 ms/帧）、1920×1080 → 17.8–19.8 s（71–79 ms/帧）、3840×2160 → 50.8 s（202 ms/帧）。三个数都受**机器负载**影响（测时负载均值 ~8，同一命令重复跑能差 30%），量级参考就行。录像**不是实时的**，但这不影响产物：MP4 的时间轴按**仿真时间**走，与机器快慢无关，分辨率越高只是录得越慢。
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

## 2026-09-25 复现上游 unitree_mujoco（C++ 与 Python 两条路线）

> 用途与任务背景见 [`../../@20260923_mujoco/README.md`](../../@20260923_mujoco/README.md) 的「复现上游参考实现」一节；复现解决的那个疑问（`LowCmd` 回调归属）见 [`../learn/runtime-timing.md`](../learn/runtime-timing.md) §10。两个自建环境都不入库，放在工作区同级目录 `Replicate.d/`：`unitree_mujoco/{python,cpp}/` 各一个 pixi 环境，与环境、版本无关的 SDK 放顶层 `Replicate.d/unitree_sdk2/` 共用。

- **两条路线各自单独建环境，不动任务主环境**。Python 侧必须 `python=3.10`：`unitree_sdk2_python/setup.py` 钉死 `cyclonedds==0.10.2`，而该版本只发布了 **cp310** 轮子，cp311/cp312 都是 0 个（`curl -s https://mirrors.ustc.edu.cn/pypi/simple/cyclonedds/ | grep -c 'cyclonedds-0.10.2.*cp312'` → `0`），在 3.12 上只能源码编 Cyclone DDS。C++ 侧不能复用主环境：官方包里的 `libmujoco.so` 会和 conda 的 `mujoco` 撞车。
- **旧 `opencv-python` 与 numpy 2 不兼容**：pip 那份 `opencv-python 4.5.5` 是按 numpy 1.x ABI 编的，在 numpy 2.2.6 下 `import cv2` 报 `numpy.core.multiarray failed to import`；把 numpy 钉到 `1.26` 即可（也符合该 SDK 的年代）。
- **C++ 路线必须另下官方 MuJoCo 发布包**：conda 的 `mujoco` 只有头文件，而 `simulate/CMakeLists.txt:25` 的 `file(GLOB …)` 要**编 `mujoco/simulate/{glfw_*,platform_*,simulate}.cc`** 这些示例源码，所以 `Replicate.d/mujoco-3.12.0/`（与 conda 同版本）免不了。
- **`unitree_sdk2` 不用自己编 Cyclone DDS、也不用 sudo**：仓库自带预编译 `lib/x86_64/libunitree_sdk2.a` 与 `thirdparty/{include,lib}` 里的 ddsc/ddscxx，`cmake --install` 到 `Replicate.d/unitree_sdk2` 即可；上游硬编码的 `/opt/unitree_robotics/lib/cmake` 是用 `list(APPEND …)` 加的（`simulate/CMakeLists.txt:12`），命令行传 `-DCMAKE_PREFIX_PATH=<install>/lib/cmake` 就能盖过它。
- **编上游 `simulate` 还差两个 conda 依赖**：glfw 的头文件 `#include <GL/gl.h>` → 需要 `libgl-devel`（conda 编译器不搜系统 `/usr/include`，即使系统装了 `libgl-dev` 也没用）；SDK 的 `include/unitree/dds_wrapper/robots/go2/go2_sub.h:7` 有 `#include <eigen3/Eigen/Dense>` → 需要 `eigen`。
- **上游按可执行文件位置找配置与场景**：`simulate/src/main.cc:683` 是 `proj_dir = getExecutableDir().parent_path()`，即要求可执行文件正好在 `simulate/` 下一层，然后读 `proj_dir/config.yaml`、场景取 `proj_dir.parent_path()/unitree_robots/<robot>/<scene.xml>`（`:684`、`:687`）。把构建目录放到 `Replicate.d` 后，用软链接补齐这套相对位置即可，参考克隆里只加一个 `simulate/mujoco → Replicate.d/mujoco-3.12.0`。
- **回环网卡上的 DDS 会打印 `selected interface "lo" is not multicast-capable: disabling multicast`**：`lo` 没有多播，属正常，单机通信仍走单播。
- **Python 侧启动前要改两个运行时条件**（不改参考源码，用启动器覆盖）：`USE_JOYSTICK = 1` 在本机没有手柄（无 `/dev/input/js*`）时会失败；`ROBOT_SCENE` 是相对路径、依赖 cwd。两者都在 `Replicate.d/unitree_mujoco/python/run_sim.py` 里改，该脚本同时负责打印线程清单与回调线程。
- **被动 viewer 需要真实窗口**：Python 侧得 `MUJOCO_GL=glfw`（主环境默认是 `egl`，无窗口）；本机 Wayland 会话下会有一条 `GLFWError: (65548) Wayland: The platform does not provide the window position` 警告，不影响显示。
- **退出阶段会段错误**：Python 侧两次复现，`viewer.close()` 之后进程以 `segmentation fault (core dumped)` 收场——上游退出路径本身缺同步，不影响前面的运行。

复现时要敲的关键命令（在 `Replicate.d/unitree_mujoco/{python,cpp}` 下执行，两个环境各自 `pixi install` 一次）：

```bash
# Python：仿真器（探针在启动器里，会打印线程清单与 LowCmd 回调所在线程）
pixi run python run_sim.py                 # 加 --seconds 10 可自动关窗退出
# 控制器另开终端（回车开始，无限循环）
printf '\n' | pixi run python ../../../ReadOnly.d/unitree_mujoco/example/python/stand_go2.py

# C++：编上游仿真器与控制器（SDK 安装前缀与官方 MuJoCo 包按上面两条准备）
SDK=/home/bis/BiS.d/Code.d/RoboCon/Replicate.d/unitree_sdk2 ; MJ=/home/bis/BiS.d/Code.d/RoboCon/Replicate.d/mujoco-3.12.0
cmake -S ../../../ReadOnly.d/unitree_mujoco/simulate -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$SDK/lib/cmake;$CONDA_PREFIX" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$CONDA_PREFIX/lib -L$MJ/lib -L$SDK/lib -Wl,-rpath,$CONDA_PREFIX/lib -Wl,-rpath,$MJ/lib -Wl,-rpath,$SDK/lib"
cmake --build build --target unitree_mujoco -j8
cmake -S ../../../ReadOnly.d/unitree_mujoco/example/cpp -B build-stand -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$SDK/lib/cmake;$CONDA_PREFIX"
cmake --build build-stand -j8
# 跑：仿真器（带官方 Simulate 界面）+ 控制器（另开终端，回车开始）
LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$SDK/lib:$MJ/lib" ./build/unitree_mujoco
LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$SDK/lib" ./build-stand/stand_go2
```

两边的运行结果（基座高度采样）见 [`../../@20260923_mujoco/README.md`](../../@20260923_mujoco/README.md) 的「复现上游参考实现」一节。
