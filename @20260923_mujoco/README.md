# MuJoCo 学习与 black 四足机器人仿真

本目录是第二次培训（MuJoCo 与机器人仿真基础）的任务工作区。 环境（Python / MuJoCo）由**仓库根目录**的 `pixi.toml` 统一管理，本目录只放模型、场景与代码。 仓库入口（任务记录、文档索引）：[`../README.md`](../README.md)。

## 运行方式

各任务做到哪一步见 `## 进度`；下面按任务分两段，都从仓库根目录执行（有 `pixi.toml` 的地方）。

### 任务 2（平地场景 + 零力矩静止趴卧）

```bash
cd ..                       # 到仓库根目录
pixi run python @20260923_mujoco/scripts/<目录>/<脚本>.py ...              # 旧入口：simulate / simulate_record / visualization / agent_scripts
pixi run python @20260923_mujoco/scripts/agent_scripts/rest_check.py       # 任务 2 的验收入口（静止判定）
pixi run cmake -S @20260923_mujoco/cpp_task2 -B @20260923_mujoco/cpp_task2/build -G Ninja -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
pixi run cmake --build @20260923_mujoco/cpp_task2/build
pixi run @20260923_mujoco/cpp_task2/build/rest_check                       # C++ 版：静止判定，同一场景、同一判据
pixi run @20260923_mujoco/cpp_task2/build/dog_sim                          # C++ 版：最小仿真（默认录像）
```

### 任务 4（新结构：Python 侧已在 `python/` 落地，C++ 侧复刻在 `cpp/`）

开窗口要临时把图形后端换回 `glfw`（仓库默认是 `egl`，无窗口）：

```bash
pixi run env MUJOCO_GL=glfw python @20260923_mujoco/python/main.py          # 开窗口，零力矩跑
pixi run python @20260923_mujoco/python/main.py --no-viewer --seconds 8     # 无窗口跑 8 仿真秒
pixi run python @20260923_mujoco/scripts/agent_scripts/physics_pacing.py    # 检查：渲染不顶住物理
```

## 目录结构

```text
@20260923_mujoco/
├── README.md
├── assets/						  # 相较于models/, scenes/，本目录下模型不在运行时运行，是静态模型（未处理初始状态是否穿模、是否是预期位姿）
│   ├── urdf/                     # 原始 URDF + meshes（真实 STL 只存这一份，34 MB）
│   ├── black_description/        # 网站导出（Floating Base ON / Torque）；meshes 为软链接
│   │   └── meshes -> ../urdf/meshes
│   └── black_description.bak/    # 旧版导出（默认 Position、无 freejoint），仅作对照，已 gitignore
├── models/
│   ├── black_description.xml     # 整理后的机器人模型（freejoint + 12 个力矩电机），唯一能跑的本体
│   └── meshes -> ../assets/urdf/meshes
├── scenes/
│   ├── flat_scene.xml            # 正常：include 上面的模型 + 地面 + 灯光 + 静止 keyframe
│   ├── flat_scene_raw.xml        # 对照：直接 include 原始导出，用于复现弹飞
│   └── meshes -> ../assets/urdf/meshes
├── output/                       # 任务结果（图像/视频；文档在仓库根的 docs/）
│   ├── preview_*.png             # render_preview.py 的截图
│   └── simulate_record.mp4       # simulate_record.py 的录像（任务 2 证据）
├── examples/                     # 跟着教程敲的小例子（与任务 2 无关）
│   └── 01_falling_box/           # 入门例：立方体落地（scene.xml + simulate.py）
├── python/                       # 任务 3：借鉴 unitree_mujoco 重塑的仿真循环（双缓冲 + 两线程）
│   ├── main.py                   # 入口：开窗口/无窗口、实时/全速
│   ├── simulator.py              # 物理线程独占 mjData；渲染只读快照副本；锁只罩 memcpy
│   └── control.py                # 控制输入：目前零力矩，键盘控制以后加在这里
├── scripts/
│   ├── simulate.py               # 自己的仿真程序（最小 viewer 循环）
│   ├── simulate_record.py        # 同上，接上录像（无窗口）
│   ├── visualization/            # 录像与截图
│   │   ├── __init__.py           # 包入口：`from visualization import VideoRecorder`
│   │   ├── mujoco_video.py       # 核心库（离屏渲染 + ffmpeg 管道）
│   │   ├── render_preview.py     # 离屏渲染单张截图
│   │   └── examples/             # 示例代码（兼零力矩录像的命令行入口）
│   │       ├── example_attach.py         # 把录像接进已有循环 + 命令行录像
│   │       ├── example_with_viewer.py    # 可选：一边开窗口看一边录
│   │       └── output/                   # 上面两个示例的产物（默认写这里）
│   ├── onetime_tools/            # 一次性工具：measure_and_fix_base_height.py（量脚底高度、抬基座）
│   └── agent_scripts/            # 诊断小工具（facts / A-B / rest_check / compare / urdf_to_mjcf / physics_pacing）
├── cpp_task2/                    # 任务 2 的 C++ 版：同一场景、同一判据，数字与 Python 侧对照
│   ├── CMakeLists.txt            # find_package(mujoco / glfw3)，工具链取自 pixi 环境
│   └── src/
│       ├── main.cpp              # 最小仿真 + 录像（对标 scripts/simulate_record.py）
│       ├── rest_check.cpp        # 零力矩静止判定（对标 scripts/agent_scripts/rest_check.py）
│       └── record.h              # 离屏渲染 → ffmpeg 的录像器（header-only）
└── cpp/                          # 任务 4 的落点：把 python/ 的双缓冲结构用 C++ 复刻（目前只有占位说明）
    └── README.md
```

## 环境与版本

| 项 | 版本 | 来源 |
|---|---|---|
| Python | 3.12.14 | conda-forge（经 pixi） |
| mujoco（Python 绑定） | 3.12.0 | conda-forge `mujoco-python` |
| libmujoco（C++ 库） | 3.12.0 | conda-forge `libmujoco` |
| mujoco-simulate（GUI） | 3.12.0 | conda-forge |
| numpy | 2.5.3 | conda-forge |

- conda-forge 的 `mujoco` **落后**于 PyPI（PyPI 已 3.14.0，conda-forge 最高 3.12.0）。 为了 C++ 与 Python 用同一份库，本项目选 conda-forge。
- 本项目走 **CPU 仿真**：MuJoCo 本体就是 CPU 引擎（没有 GPU 版本），GPU 只影响 **渲染 / MJX / MJWarp** 三条支线，本项目用的是“渲染”这条（见下面“录像工具”）。 为什么本机用不了 MJX / MJWarp（显卡算力不够），见 [`../docs/pitfalls/environment.md`](../docs/pitfalls/environment.md) 的「图形后端 / GPU / 渲染性能」一节。

## URDF 来源

原始文件：`N-W-wolf/Training_Materials@main: 第二次培训/black/black_description.urdf` （`N-W-wolf` 下的四足组培训资料仓库；`rl_sar-black-W` 中另有一份不同版本的 `src/robots/black_description/urdf/black_description.urdf`）。

本目录 `assets/urdf/` 中的副本已用 sha256 校验与上游一致： `e337a9d619154b98c0484bf4c41e20dd44b48c877d64916a16cb664b4cd12f26`。

## 关键结论：URDF → MJCF 的两条路线

URDF→MJCF 有两条现成路线：网站（urdf.enkeebot.com）导出，以及用 MuJoCo 自带的 `mj_saveLastXML` / `MjSpec.to_xml` 转。**两者的产物差别很大**，本任务用的是前者；下面列清差异，便于以后再换模型时判断该走哪条（两条路线都还需要自己组装平地场景与自由基座，都不是“开箱可跑”）。

| 模型 | 体积(B) | nbody | njnt | 自由关节 | nq | nv | nu | ngeom | nmesh | nmat | 总质量(kg) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `black_description.urdf`（MuJoCo 直接读） | 18765 | 13 | 12 | 0 | 12 | 12 | 0 | 30 | 0 | 0 | **7.4700** |
| 网站旧导出 `…skeleton.xml`（现 .bak） | 9378 | 20 | 13 | **1** | 19 | 18 | 12 | 19 | 14 | 0 | 13.2472 |
| 网站旧导出 `…description.xml`（现 .bak，默认选项） | 17971 | 20 | **12** | **0** | 12 | 12 | 12 | 49 | 14 | 19 | 13.2472 |
| 网站新导出 `black_description.xml`（Floating Base ON + Torque） | 17942 | 20 | 13 | **1** | 19 | 18 | 12 | 49 | 14 | 19 | 13.2472 |
| MuJoCo `saveLastXML` / `spec` | 7786 | 13 | 12 | 0 | 12 | 12 | 0 | 30 | 0 | 0 | **7.4700** |

（复现：`pixi run python scripts/agent_scripts/compare_mjcf.py <文件...>`）

表里最后一行（MuJoCo 自带转换的产物）**不入库**：它可由脚本一键复现，而且两种方法 的输出**逐字节相同**（sha256 `8bc83581660f9d494f932c3795446c0ad1fb66c2296e68feae78677970a58d2e`）：

```bash
pixi run python @20260923_mujoco/scripts/agent_scripts/urdf_to_mjcf.py \
    @20260923_mujoco/assets/urdf/black_description.urdf /tmp/converted.xml   # 也可加 --method spec
```

**MuJoCo 自带转换的特点：**

1. 只导入 URDF 的 `<collision>`，**完全丢弃 `<visual>` mesh**，因此没有 `<asset>`、没有材质；
2. **根 link（`trunk`）被并入 `worldbody`**：机器人的基座被"焊死"在世界上，还丢掉了
   基座的质量与惯量（总质量 13.2472 → 7.47 kg，差值 5.7772 kg 正好是 trunk+imu_link+d435_link）——用于四足仿真时必须自己补一个带 `<freejoint/>` 的 body；
3. `fixed` 关节的子 link 被合并（4 个 `*_foot` 的质量并入对应 `*_calf`）；
4. 没有 actuator（URDF 里本来也没有执行器概念）；
5. 会把 URDF 的 `effort` 限制转成 `actuatorfrcrange`。

**网站转换的特点：**

1. 视觉 mesh 作为 `group="1"`、`contype=0 conaffinity=0 mass=0` 的"只显示不碰撞"geom 保留；
   collision 作为 `group="3"` 的 box/cylinder/sphere；
2. 每个 link 的 `<inertial>` 完整保留 ⇒ 总质量与 URDF 一致；
3. **默认选项的导出**（现 `assets/black_description.bak/black_description.xml`）会生成
   12 个 `<position>`（位置伺服，kp=10）、**且没有 `<freejoint/>`**；
4. `…_skeleton.xml`（现也在 `.bak/`）是“去掉碰撞体的骨架版”：带 `<freejoint name="trunk"/>`、
   `<default>` 里是 `<motor ctrlrange="-1 1"/>`（力矩模式，但 ±1 N·m 太小）， 且**所有 geom 都无碰撞**，只能当参考模板。
5. **按正确选项重新导出**（Floating Base ON + Torque + 骨架关）得到现在的
   `assets/black_description/black_description.xml`：freejoint + 12 个 `<motor gear="1" ctrlrange="-20 20">` + 完整视觉/碰撞几何 + 材质，总质量 13.2472 kg。

**结论**：本任务选网站导出（几何/惯量保真度明显更好：保留 mesh 与原始惯性）；两条路线都**不会**自动给出能直接跑的“平地 + 自由基座 + 力矩电机”模型，这部分需要自己组装（见 `scenes/` 与 `models/`）。

## 脚本

| 脚本 | 作用 |
|---|---|
| `scripts/simulate.py` | 自己的仿真程序：最小 `launch_passive` 循环（不录像） |
| `scripts/simulate_record.py` | 同上 + 录像（无窗口）；就是“只加一行 `rec.capture(data)` 就接上了”的样子 |
| `scripts/visualization/__init__.py` | 包入口：`from visualization import VideoRecorder`（把库的常用名字提到包一级） |
| `scripts/visualization/mujoco_video.py` | **录像工具库**：离屏取帧 → 管道给 ffmpeg(libx264)；`VideoRecorder` 可插进任意已有循环。**MuJoCo 没有原生录像功能** |
| `scripts/visualization/render_preview.py` | 离屏渲染单张截图（PNG 用标准库写出，不需要 pillow） |
| `scripts/visualization/examples/example_attach.py` | **接入示例**，同时是**零力矩录像的命令行入口**（原 `record_scene.py` 的职责已并入此处）：照抄 `simulate.py` 的循环形状，只多 `rec.capture(data)` 一行（不开窗口）；`--scene/--start/--camera/--follow/--fps/--width/--height` 可调 |
| `scripts/visualization/examples/example_with_viewer.py` | 可选：一边开 `launch_passive` 看一边录（含 `viewer.sync()` 开销与实时节流的实测结论） |
| `scripts/onetime_tools/measure_and_fix_base_height.py` | 把网站导出整理成 `models/black_description.xml`（1 条手工补丁 + 断言 + 软链接自愈，可复现） |
| `scripts/agent_scripts/mujoco_facts.py` | 打印 geom type / friction / condim 的实测结论，供 `../docs/learn/mujoco.md` 引用 |
| `scripts/agent_scripts/ab_initial_state.py` | A/B 对照：原始导出 vs 补丁后模型的初始状态（穿模 / 弹飞量化） |
| `scripts/agent_scripts/rest_check.py` | 任务 2 验证：`--mode drop` 求趴卧姿态，`--mode keyframe` 验证零力矩静止 |
| `scripts/agent_scripts/compare_mjcf.py` | 打印多个模型的结构指标，用于对比转换结果 |
| `scripts/agent_scripts/urdf_to_mjcf.py` | 用 MuJoCo 自带能力把 URDF 转成 MJCF（`--method saveLastXML\|spec`） |

## 网站导出选项怎么选（实测结论）

`urdf.enkeebot.com` 的选项里，只要调对三个开关，就能得到「可直接用」的模型：

| 选项 | 取值 | 为什么 |
|---|---|---|
| Floating Base | **开** | 否则根 body 被焊死在 world（nq=12、无 `<freejoint/>`） |
| Actuator Type | **Torque** | 得到 `<motor ctrlrange="-20 20" gear="1">`（力矩模式，限幅已与 URDF `effort=20` 一致）；Position 会给 `<position kp=10>` |
| Include Skeleton | **关** | 骨架版 **19 个 geom 全是 `contype=0 conaffinity=0`（完全无碰撞）**，拿它做“趴在地上”会直接穿过地面 |
| 其余（Mesh Directory / Mesh Format / STL Quality / Shared Mesh Reuse） | 默认 | 保持相对路径与最高保真 |

导出后需要手工处理的只剩**一件事**，已在 `scripts/onetime_tools/measure_and_fix_base_height.py` 里固化成可复现补丁：

1. 基座默认高度 `pos="0 0 0"` → `pos="0 0 0.578580"`，理由见下面“初始穿模”。
   这个高度**不是硬编码常数**：脚本用脚底球体几何实时算出，并在最后重新加载产物 断言“默认状态脚底 z ≥ 0”。

**`meshdir` 不改**：导出自带 `meshdir="meshes/"`，网格重复问题用**目录软链接**解决： 真实 STL 只在 `assets/urdf/meshes` 存一份，每个会用到它的目录各放一个 `meshes` 软链接 （`assets/black_description/`、`models/`、`scenes/`）。`measure_and_fix_base_height.py` 会把这三个链接 建好（缺失就补，指向不对就报错）。为什么 `scenes/` 也要一个，见下面 5.1 那条坑。

场景（地面/灯光）仍需自己写 —— 见 `scenes/flat_scene.xml`（正常仿真）与 `scenes/flat_scene_raw.xml`（对照用，直接 include 原始导出）。

### 踩坑 5.1：`<include>` 之后 `meshdir` 是相对**顶层文件**解析的

场景与机器人分目录时，模型里的 `meshdir="meshes/"` **不会**相对模型自己解析，而是相对 顶层（scene）文件所在目录。实测：`scenes/flat_scene.xml` 去 include 模型、模型写 `meshdir="meshes/"` → 报错去找 `scenes/meshes/...`；而**同一个模型单独加载却正常**。

所以场景目录旁也必须有一个 `meshes` 软链接（这就是为什么三个目录都有）。 另外注意：`<keyframe>` 不会被自动加载，最简 `viewer.launch_passive` 循环拿到的是模型 默认位形。

### 踩坑 5.2：默认位形就穿模，求解器会把狗弹飞

导出把 `trunk` 放在 `pos="0 0 0"`，而零位形下**脚底在基座下方 0.5786 m**。于是任何 “建完 `MjData` 直接 `mj_step`”的脚本（最简 viewer 程序就是这样，**场景里的 keyframe 不会被自动加载**）拿到的初始状态是：整只狗以地面为中心，四条腿扎进地里 0.58 m。 接触求解器会在第一步释放巨大的穿透恢复力，表现就是**狗被一下弹到空中**。

修好之后两种起手方式都可用（模型已把基座抬到触地高度，所以不用额外动作）：

* 默认位形 → 四脚站在地面上，零力矩下**自然塌成趴卧**（`examples/example_attach.py` 默认录的就是这一段）；
* 想一开始就是静止趴卧 → `mujoco.mj_resetDataKeyframe(model, data, 0)` （对应 `scenes/flat_scene.xml` 里的 `<keyframe name="rest">`）。

**关于这个 keyframe**（2026-09-25 复核）：它还在、也还有用——`rest_check.py` 的默认模式、 `example_attach.py --start rest`、`render_preview.py` 都在读它；而重跑一次自由落体 （`rest_check.py --mode drop`）得到的 qpos 与它**逐位一致**，说明没有过期。 它只写了 `qpos`（`qvel` 默认 0），且 xy 清零（平面上平移等价）。 **什么时候要重做**：改了模型的惯性/几何/执行器，或动了 `timestep`/`solver` 导致平衡位形变化时， 重跑 `rest_check.py --mode drop` 把新 qpos 粘回去；默认模式（`keyframe`）就是它的回归测试 （要求末段 max|qvel| < 1e-3 且漂移 < 1 mm）。

另外，`inertiafromgeom="auto"` 这里**刻意不改**：它只在 body 没有显式 `<inertial>` 时才 生效，而本模型 20 个 body 全都有 ⇒ 改了也不会有任何行为差异，属于纯防御性改动， 只会让补丁清单变长、diff 变脏。

## 任务 2 结果：平地 + 零力矩静止趴卧

```bash
cd ..                                # 仓库根目录（有 pixi.toml）
pixi run python @20260923_mujoco/scripts/agent_scripts/rest_check.py              # 验证 keyframe 静止
pixi run python @20260923_mujoco/scripts/agent_scripts/rest_check.py --mode drop  # 重新求趴卧姿态
pixi run python @20260923_mujoco/scripts/visualization/examples/example_attach.py  # 录像（需要 ffmpeg）
pixi run python @20260923_mujoco/scripts/simulate_record.py                      # 自己的循环 + 录像
```

实测（`timestep=0.002`，`ctrl` 全 0）：

| 阶段 | 结果 |
|---|---|
| 站立姿态放到地面 | 脚底最低点相对基座 -0.578580 m，基座 z=0.5786 m |
| **默认位形自由落下 8 s**（最简脚本的走法） | 自然塌成四肢收拢的趴卧姿态：基座 z=**0.1449 m**，8 个接触点，末段 max\|qvel\|=3.45e-08 |
| 从收敛姿态（keyframe）重新开始 8 s | **总漂移 0.0000 m，末 1 s 漂移 4.4e-10 m，末段 max\|qvel\|=3.6e-08** → 静止趴住 ✓ |

关键点：**初始姿态不是拍脑袋写的常数**，而是先由脚底球体几何反算出“脚刚好触地”的基座高度， 让它自由落下收敛，再把收敛后的 qpos 写回 `scenes/flat_scene.xml` 的 `<keyframe name="rest">`， 这样模型一加载就已经是静态平衡位形，不需要“先掉一下”。

**A/B 对照**（同一平地、模型默认位形、`ctrl=0`、2 s）—— 证明“穿模”确实是弹跳的根因：

| 场景 | 初始脚底 z | 初始基座 z | 2 s 内最高基座 z | 2 s 内最高 \|qvel\| |
|---|---|---|---|---|
| `flat_scene_raw.xml`（原始导出，`trunk pos="0 0 0"`） | **-0.5786** | 0.0000 | **4.0444** | **49.36** |
| `flat_scene.xml`（补丁后） | -0.0000 | 0.5786 | 0.5786 | 15.86 |

对照脚本：`pixi run python @20260923_mujoco/scripts/agent_scripts/ab_initial_state.py`。

录像与截图：

| 文件 | 内容 |
|---|---|
| `output/simulate_record.mp4` | `scripts/simulate_record.py`：自己的循环 + 录像（无窗口、50 fps、4.0 s） |
| `scripts/visualization/examples/output/example_attach.mp4` | `example_attach.py`：无窗口、50 fps，把录像接进一个“已有”循环 |
| `scripts/visualization/examples/output/record_with_viewer.mp4` | `example_with_viewer.py`：一边开窗口一边录（10 fps，否则跟不上实时） |
| `output/preview_iso.png` / `output/preview_side.png` | 静态截图 |
| `output/cpp_record.mp4` | `cpp_task2` 的 `dog_sim`（默认录像）：C++ 离屏渲染 → ffmpeg（4 仿真秒、50 fps、**191 帧 / 3.82 s / 960×540**） |

## 录像工具：怎么接入自己的仿真脚本

最早的 `record_video.py` 自带一个仿真循环（想录自己的控制逻辑就只能把逻辑抄进去）。现在**录**与**算**是分开的： 真正干活的是库 `scripts/visualization/mujoco_video.py` —— 它不关心你的循环长什么样，只要模型/数据 是你自己的，并每个仿真步调一次 `capture(data)`（`visualization/__init__.py` 把它提成了包入口， 所以 `scripts/` 下的脚本直接 `from visualization import VideoRecorder` 就行）：

```python
from visualization import VideoRecorder                   # ← ① 新增
...
with VideoRecorder(model, "out.mp4", fps=50, camera="iso") as rec:   # ← ② 新增
    while 你的条件:
        你的控制逻辑                                      # 一行都不用改
        mujoco.mj_step(model, data)
        rec.capture(data)                                # ← ③ 新增
```

现成可照抄的模板：`scripts/simulate_record.py`（就是 `simulate.py` 接上录像的版本，无窗口）。

* **不需要窗口**：离屏渲染走 EGL（仓库根 `pixi.toml` 的 `[activation.env]` 已设 `MUJOCO_GL=egl`）。 这个变量必须**在 `import mujoco` 之前**生效，脚本里再 `setdefault` 可能已经太晚； 不设时的默认值（`glfw`）依赖显示服务，而且在双显卡机器上还可能落到另一块 GPU 上 —— 实测见 [`../docs/learn/mujoco.md` 第 7 节](../docs/learn/mujoco.md)。 图形栈背景（EGL / GLFW / Skia 各在哪一层）见 [`../docs/learn/graphics-stack.md`](../docs/learn/graphics-stack.md)。
* 帧率、相机、编码、文件收尾全在库里；`capture()` 按**仿真时间** `data.time` 决定该不该出帧， 所以输出 MP4 的时间轴 = 仿真时间，改 `fps=` 不用动循环，**与渲染耗时/机器快慢无关** （慢机器只是录得久，产物一模一样）。
* **分辨率是独立可调的**：`--width/--height`，或库里 `VideoRecorder(width=…, height=…)`。 视角由相机决定、与像素数无关，所以“同样的画面、更多像素”是原生支持的； 各档分辨率的单帧耗时实测见 [`../docs/learn/mujoco.md` 第 7.3 节](../docs/learn/mujoco.md)。
* 完整的“接入”示例：`example_attach.py`（照抄 `simulate.py` 的循环形状，只多一行）。
* 想顺便在屏幕上开窗口看，才需要 `example_with_viewer.py`：`viewer.sync()` 每次都要等一个 显示刷新周期，所以必须**降频**（每 10~25 步一次），否则仿真速度会被显示刷新钉住。 实测数据（本机）见 [`../docs/learn/mujoco.md` 第 7 节](../docs/learn/mujoco.md)。

## C++ 程序

### 任务 2 的 C++ 版（`cpp_task2/`）

与 Python 侧**共用同一个 pixi 环境**和**同一份 MJCF**（`models/` + `scenes/`），所以两边算出的数字可以直接对照。两个可执行文件，分别对标两个 Python 脚本：

| 可执行文件 | 对标 | 干什么 |
|---|---|---|
| `rest_check` | `scripts/agent_scripts/rest_check.py` | 读场景自带的 `rest` keyframe、零力矩跑 N 秒，打印基座漂移/末段 max\|qvel\|/接触点数；静止退出码 0、否则 2 |
| `dog_sim` | `scripts/simulate_record.py` | 不加载 keyframe（默认位形自然塌成趴卧）、零力矩跑 N 秒（默认 4 s），**默认顺带录像** |

```bash
pixi run cmake -S @20260923_mujoco/cpp_task2 -B @20260923_mujoco/cpp_task2/build -G Ninja -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
pixi run cmake --build @20260923_mujoco/cpp_task2/build
pixi run @20260923_mujoco/cpp_task2/build/rest_check                                                 # 静止判定（默认 8 s），退出码 0 = 静止
pixi run @20260923_mujoco/cpp_task2/build/rest_check @20260923_mujoco/scenes/flat_scene_raw.xml 2   # 反例：穿模被弹飞，判“未静止”
pixi run @20260923_mujoco/cpp_task2/build/dog_sim                                                    # 默认 4 s，录到 output/cpp_record.mp4
pixi run @20260923_mujoco/cpp_task2/build/dog_sim @20260923_mujoco/scenes/flat_scene.xml 3 --no-record   # 只跑不录；另有 --out/--fps/--width/--height/--camera
```

实测（8 s、`ctrl=0`）：末 1 s xy 漂移 4.440e-10 m、末态 max|qvel| 6.06e-09、接触点数 8，与 Python 侧 `rest_check.py` **同一判据**（漂移 4.440e-10 m、ncon=8）。`cpp_task2/build/` 是构建产物、已被 `.gitignore` 忽略。

**录像**（`dog_sim` 默认就录，对应 Python 侧的 `VideoRecorder`）：离屏渲染（隐藏窗口拿 GL 上下文）→ `mjr_readPixels` → ffmpeg 管道，代码在 `cpp_task2/src/record.h`；出帧按 **仿真时间** 决定，所以 MP4 的时间轴 = 仿真时间、与机器快慢无关；`--no-record` 可关，`--out/--fps/--width/--height/--camera` 可调。实测 4 仿真秒 @50 fps → `output/cpp_record.mp4`：**191 帧 / 3.82 s / 960×540**。

### 任务 4 的落点（`cpp/`）

`cpp/` 只放任务 4 的实现：把 [`python/`](python/) 的双缓冲结构用 C++ 复刻（物理线程独占 `mjData`、渲染只读快照副本、锁只罩 memcpy），之后再接官方 `Simulate` 界面或自带渲染循环。目前那里只有一份 `README.md` 占位。

**为什么做 C++**：不是为了更快——`mj_step` 两边调用的是同一份 C 库，单步耗时几乎一样（实测数据见 [`../docs/pitfalls/environment.md`](../docs/pitfalls/environment.md) 的「C++ 工具链」一节），渲染开销也只由 GPU 决定；意义在**工程结构与 sim-to-real**（真实机器人上的控制程序是 C++）。工具链选择（为什么用 pixi 的编译器、编辑器提示怎么配）同样记在 [`../docs/pitfalls/environment.md`](../docs/pitfalls/environment.md)。

## 复现上游参考实现（任务 3 的前置验证）

为了把「`LowCmd` 回调到底跑在哪个线程」从“读代码推断”变成运行期事实，也为了先把参考实现的**预期效果**摸清、给后面的 Python 重构留一个对照基准，2026-09-25 在本机把上游 `unitree_mujoco` 的 `simulate_python/`、`simulate/`（C++）连同 `example/{python,cpp}/stand_go2.*` 都跑通了一遍。

两个复现环境**不在本仓库内**，放在工作区同级的 `Replicate.d/`（相对本目录是 `../../Replicate.d/`）：参考克隆仍是 `ReadOnly.d/unitree_mujoco`，只往里加了一个指向官方 MuJoCo 包的软链接，源代码未改。为什么必须另建环境（Python 必须是 3.10、`cyclonedds==0.10.2` 只有 cp310 轮子、C++ 侧为何必须另下官方 MuJoCo 包、需要 `libgl-devel` 与 `eigen`、上游按可执行文件位置找配置与场景、退出时段错误）全部记在 [`../docs/pitfalls/environment.md`](../docs/pitfalls/environment.md) 的「复现上游 unitree_mujoco」一节，连完整命令一起。

两个终端各跑一边（仿真器与控制器通过域 1 的 DDS 在 `lo` 上通信，控制器都要按一次回车才开始）：

```text
Python：Replicate.d/unitree_mujoco/python 下 pixi run python run_sim.py，另开终端跑 example/python/stand_go2.py
C++   ：Replicate.d/unitree_mujoco/cpp 下跑 ./build/unitree_mujoco（带官方 Simulate 界面），另开终端跑 ./build-stand/stand_go2
```

实测的预期效果（`stand_go2` 的脚本是先站起、3 秒后再趴下；基座高度取自仿真器发布的 `rt/sportmodestate`）：

| 实现 | 起始（无控制稳态） | 站起峰值 | 最终（趴下稳态） |
|---|---|---|---|
| Python（1 s 采样 `qpos[:3]`） | 0.0771（自由落体后瘫地） | 0.3432 | 0.1352 |
| C++（1 s 采样 `rt/sportmodestate`） | 0.1797（上一次运行留下的蹲姿） | 0.3897 | 0.1776 |

两个数不一致本身就是发现：两边时间步不同（Python `SIMULATE_DT = 0.005`、C++ `0.002`），伺服收敛位形因此不同；做 A/B 对比时不能直接把两边的绝对高度拿来比。回调归属、线程清单与复核命令见 [`../docs/learn/unitree-mujoco-threads.md`](../docs/learn/unitree-mujoco-threads.md) §10。

## 进度

- [ ] 任务 1：认识 MuJoCo（作用、Python 接口、MJCF 结构）
- [x] 任务 2：URDF→MJCF、平地场景、零力矩静止趴卧、力矩执行器
- [ ] 任务 3：参考 unitree_mujoco 优化代码结构与线程设计（研读笔记 → [`../docs/learn/unitree-mujoco.md`](../docs/learn/unitree-mujoco.md)，线程/通信细节 → [`../docs/learn/unitree-mujoco-threads.md`](../docs/learn/unitree-mujoco-threads.md)；**Python 侧已落地**：[`python/`](python/) 用双缓冲把渲染与物理拆开。实测（`scripts/agent_scripts/physics_pacing.py`）：同等 20 ms/次渲染下，无窗口我们 499 步/秒（实时 0.998x）、上游式单锁写法 271 步/秒（0.542x），且物理结果与单线程裸循环逐位相同；开窗口时降到 0.863x——那是 Python 的 GIL 争用（渲染那一步在 Python 里），不是锁，留给任务 4 用 C++ 解决）
- [ ] 任务 4（选做）：用 C++ 重做（任务 2 的 C++ 版已落到 [`cpp_task2/`](cpp_task2/)；任务 4 的双缓冲结构待在 [`cpp/`](cpp/) 实现，对齐 [`python/`](python/)）

任务 3/4 的推进顺序：① C++ 工具链可行性验证（已完成）→ ② 研读 `unitree_mujoco`、写 `docs/learn/unitree-mujoco.md`（已完成）→ ③ Python 侧按新结构重构（**已完成**：`python/`，`scripts/` 里的旧脚本暂留作对照）→ ④ C++ 复刻同一结构（先无窗口 + 录制，再接官方 `Simulate` 界面）。
