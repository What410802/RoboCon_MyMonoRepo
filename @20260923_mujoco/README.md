# MuJoCo 学习与 black 四足机器人仿真

本目录是第二次培训（MuJoCo 与机器人仿真基础）的任务工作区。环境（Python / MuJoCo / C++ 工具链）由**仓库根目录**的 `pixi.toml` 统一管理，本目录只放模型、场景与代码。仓库入口（任务记录、文档索引）：[`../README.md`](../README.md)。

## 文档

分工按 [`../docs/conventions.md`](../docs/conventions.md) §7：**本目录的文档只写本任务成立的事**（模型来源、本任务的过程与产出、本任务的数字），**通用知识与环境依据放仓库 `docs/`**，两边不重复、父文档只放飞指针。

| 文档 | 内容 |
|---|---|
| [`docs/model.md`](docs/model.md) | URDF 来源、URDF→MJCF 两条路线的差异、网站导出选项、本模型的两条坑（`meshdir` / 初始穿模）、`rest` keyframe 的来龙去脉 |
| [`docs/task2.md`](docs/task2.md) | 任务 2 的结果与 A/B 对照、录像与截图产物 |
| [`docs/recording.md`](docs/recording.md) | 录像工具怎么接进自己的仿真循环（Python 侧 `VideoRecorder`） |
| [`docs/replication.md`](docs/replication.md) | 复现上游 `unitree_mujoco` 的过程记录与预期效果 |

通用部分（MuJoCo 知识点与坑、图形栈、录像/渲染开销、编辑器提示、环境与镜像）在仓库 [`../docs/`](../docs/) 下。

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
pixi run @20260923_mujoco/cpp_task2/build/dog_sim                          # C++ 版：最小仿真（默认开官方 Simulate 窗口）
```

### 任务 4（新结构：Python 侧已在 `python/` 落地，C++ 侧复刻在 `cpp/`）

开窗口要临时把图形后端换回 `glfw`（仓库默认是 `egl`，无窗口）：

```bash
pixi run env MUJOCO_GL=glfw python @20260923_mujoco/python/main.py          # 开窗口，零力矩跑
pixi run python @20260923_mujoco/python/main.py --no-viewer --seconds 8     # 无窗口跑 8 仿真秒
pixi run python @20260923_mujoco/scripts/agent_scripts/physics_pacing.py    # 检查：渲染不顶住物理
```

**为什么非阻塞（双缓冲）是必要的**：`scripts/simulate.py` 是最朴素的单线程写法——一圈里 `mj_step` 之后紧跟 `viewer.sync()`，而 `viewer.sync()` 要等一个刷新周期（本机实测中位 **23.1 ms**）≫ `timestep`（0.002 s），一圈只推进 0.002 s 仿真，整个循环被显示刷新钉住。实测（本机 i5-1035G1、960×540、`MUJOCO_GL=glfw`）：

| 结构 | 开销 | 实时率 |
|---|---|---|
| 纯 `mj_step`（无窗口） | 0.0432 ms/步（C++ 侧 0.0404 ms） | — |
| `scripts/simulate.py`：`mj_step` + `viewer.sync()` 同线程、**每步都 sync** | 23.1 ms/圈，一圈才走 0.002 s | **0.089x** |
| `python/main.py`：物理线程 + 渲染线程、锁只罩快照 memcpy、deadline pacing | 渲染 20 ms/次也不顶住物理 | **0.998x**（开窗口约 0.86x，GIL 限制） |
| `cpp_task2 --mode view`：物理线程 + 官方 `Simulate` 界面 | 官方 `RenderLoop` 在 `Render()` **之前**就放锁（源码注释 `// MutexLock (unblocks simulation thread)`） | **1.00x** |

也正因为如此，`scripts/simulate.py` 里那句被注释掉的 `[WARN] Simulation speed decreased.` 在本机是常态（每圈都会触发）；上游把 `SIMULATE_DT` 放大到 0.005 s 正是在迁就这件事（`config.py:13` 的注释明说）。**结论：要一边按墙钟实时看、一边推进物理，非阻塞（快照 + 双缓冲）不是可选优化，而是必要条件**——纯物理本身就够快（0.04 ms/步），问题全在「谁在等谁」。

## 目录结构

```text
@20260923_mujoco/
├── README.md                     # 本文件：入口（做了什么、怎么跑、结果在哪）
├── docs/                         # 本任务的文档（分工见上）
│   ├── model.md                  # 模型来源与 URDF→MJCF 转换（含本模型的两条坑）
│   ├── task2.md                  # 任务 2 结果、A/B 对照、录像产物
│   ├── recording.md              # 录像工具怎么接进自己的循环
│   └── replication.md            # 复现上游 unitree_mujoco 的记录
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
├── output/                       # 任务结果（按语言分；文档在仓库根的 docs/）
│   ├── python/                   # Python 侧：simulate_record.mp4、preview_*.png、example_attach.mp4、record_with_viewer.mp4
│   └── cpp/                      # C++ 侧：cpp_record.mp4（每项产物的产出命令见 docs/task2.md）
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
│   │       └── example_with_viewer.py    # 可选：一边开窗口看一边录（默认也写 output/python/）
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

- **本任务不需要额外依赖，也不需要改环境**：直接用仓库根 `pixi.toml` 装出来的那一个环境（怎么搭出来、头文件是谁装的、镜像与工具链的取舍见 [`../docs/pitfalls/environment.md`](../docs/pitfalls/environment.md)）。
- conda-forge 的 `mujoco` **落后**于 PyPI（PyPI 已 3.14.0，conda-forge 最高 3.12.0）。为了 C++ 与 Python 用同一份库，本项目选 conda-forge。
- 本项目走 **CPU 仿真**：MuJoCo 本体就是 CPU 引擎（没有 GPU 版本），GPU 只影响 **渲染 / MJX / MJWarp** 三条支线，本项目用的是“渲染”这条。为什么本机用不了 MJX / MJWarp（显卡算力不够），见 [`../docs/pitfalls/environment.md`](../docs/pitfalls/environment.md) 的「图形后端 / GPU / 渲染性能」一节。

## 脚本

| 脚本 | 作用 |
|---|---|
| `scripts/simulate.py` | 自己的仿真程序：最小 `launch_passive` 循环（不录像） |
| `scripts/simulate_record.py` | 同上 + 录像（无窗口）；就是“只加一行 `rec.capture(data)` 就接上了”的样子 |
| `scripts/visualization/__init__.py` | 包入口：`from visualization import VideoRecorder`（把库的常用名字提到包一级） |
| `scripts/visualization/mujoco_video.py` | **录像工具库**：离屏取帧 → 管道给 ffmpeg(libx264)；`VideoRecorder` 可插进任意已有循环。用法见 [`docs/recording.md`](docs/recording.md) |
| `scripts/visualization/render_preview.py` | 离屏渲染单张截图（PNG 用标准库写出，不需要 pillow） |
| `scripts/visualization/examples/example_attach.py` | **接入示例**，同时是**零力矩录像的命令行入口**：照抄 `simulate.py` 的循环形状，只多 `rec.capture(data)` 一行（不开窗口）；`--scene/--start/--camera/--follow/--fps/--width/--height` 可调 |
| `scripts/visualization/examples/example_with_viewer.py` | 可选：一边开 `launch_passive` 看一边录（含 `viewer.sync()` 开销与实时节流的实测结论） |
| `scripts/onetime_tools/measure_and_fix_base_height.py` | 把网站导出整理成 `models/black_description.xml`（1 条手工补丁 + 断言 + 软链接自愈，可复现） |
| `scripts/agent_scripts/mujoco_facts.py` | 打印 geom type / friction / condim 的实测结论，供 [`../docs/learn/mujoco.md`](../docs/learn/mujoco.md) 引用 |
| `scripts/agent_scripts/ab_initial_state.py` | A/B 对照：原始导出 vs 补丁后模型的初始状态（穿模 / 弹飞量化） |
| `scripts/agent_scripts/rest_check.py` | 任务 2 验证：`--mode drop` 求趴卧姿态，`--mode keyframe` 验证零力矩静止 |
| `scripts/agent_scripts/compare_mjcf.py` | 打印多个模型的结构指标，用于对比转换结果 |
| `scripts/agent_scripts/urdf_to_mjcf.py` | 用 MuJoCo 自带能力把 URDF 转成 MJCF（`--method saveLastXML\|spec`） |
| `scripts/agent_scripts/physics_pacing.py` | 任务 3 检查：渲染开销不该影响物理步进（新循环 vs 上游式单锁写法） |

## C++ 程序

### 任务 2 的 C++ 版（`cpp_task2/`）

与 Python 侧**共用同一个 pixi 环境**和**同一份 MJCF**（`models/` + `scenes/`），所以两边算出的数字可以直接对照。两个可执行文件，分别对标两个 Python 脚本：

| 可执行文件 | 对标 | 干什么 |
|---|---|---|
| `rest_check` | `scripts/agent_scripts/rest_check.py` | 读场景自带的 `rest` keyframe、零力矩跑 N 秒，打印基座漂移/末段 max\|qvel\|/接触点数；退出码：0 = 静止趴住、2 = 判“未静止”、1 = 参数写错 |
| `dog_sim` | `scripts/simulate.py` / `simulate_record.py` | 不加载 keyframe（默认位形自然塌成趴卧）、零力矩跑 N 秒（默认 4 s）；三种模式：`--mode sim` 只仿真（无窗口无录像，全速）、`--mode record` 离屏录像、`--mode view`（**默认**）开 MuJoCo 官方 Simulate 窗口（时长不限，关窗结束） |

```bash
pixi run cmake -S @20260923_mujoco/cpp_task2 -B @20260923_mujoco/cpp_task2/build -G Ninja -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
pixi run cmake --build @20260923_mujoco/cpp_task2/build
pixi run @20260923_mujoco/cpp_task2/build/rest_check                                                 # 静止判定（默认 8 s），退出码 0 = 静止
pixi run @20260923_mujoco/cpp_task2/build/rest_check @20260923_mujoco/scenes/flat_scene_raw.xml 2   # 反例：穿模被弹飞，判“未静止”
pixi run @20260923_mujoco/cpp_task2/build/dog_sim                                                    # 默认：开官方 Simulate 窗口（关窗结束）
pixi run @20260923_mujoco/cpp_task2/build/dog_sim --mode record                                      # 离屏录像，录到 output/cpp/cpp_record.mp4
pixi run @20260923_mujoco/cpp_task2/build/dog_sim @20260923_mujoco/scenes/flat_scene.xml 3 --mode sim # 只仿真（无窗口无录像）
```

两个程序的命令行都**不接受认不出的选项**：写错就直接报错退出（退出码 1）并打印用法，不会把 `--p` 这种未知选项默默当成场景路径；数值参数（`seconds`、`--fps/--width/--height`）不是数字、`--out` 这类缺值、`--mode` 不是 sim/record/view、位置参数超过两个也都同样报错。两个程序的解析逻辑都集中在 `cpp_task2/src/args.h`（`OptionDef` + `Args` + `ParseArgs`），主程序只声明自己认哪些选项、再取值。

实测（8 s、`ctrl=0`）：末 1 s xy 漂移 4.440e-10 m、末态 max|qvel| 6.06e-09、接触点数 8，与 Python 侧 `rest_check.py` **同一判据**（漂移 4.440e-10 m、ncon=8）。产物清单见 [`docs/task2.md`](docs/task2.md)。

**录像**（`--mode record`，对应 Python 侧的 `VideoRecorder`）：离屏渲染（隐藏窗口拿 GL 上下文）→ `mjr_readPixels` → ffmpeg 管道，代码在 `cpp_task2/src/record.h`；出帧按 **仿真时间** 决定，所以 MP4 的时间轴 = 仿真时间、与机器快慢无关；`--out/--fps/--width/--height/--camera` 可调。实测 4 仿真秒 @50 fps → `output/cpp/cpp_record.mp4`：**191 帧 / 3.82 s / 960×540**。离屏缓冲实际是 1280×720 这件事见 [`../docs/pitfalls/environment.md`](../docs/pitfalls/environment.md) 的图形后端一节。

**窗口模式**（`--mode view`，默认）：链的是 conda 包里 MuJoCo 自带的官方界面库（`mujoco::libmujoco_simulate`，即 `mj::Simulate` + `mj::GlfwAdapter`），所以窗口与 Python 的默认窗口、`unitree_mujoco` 的 C++ 窗口**是同一个界面**：能暂停/单步/调速/换相机/拖动物体。起点与另两种模式一致（不加载 keyframe，零力矩从默认位形塌成趴卧）；物理线程把仿真时间轴钉在墙钟上（速度取界面上的下拉框，**上限 100%、不会比实时快**；实际倍率写回界面的 Real-time 显示），主线程跑 `RenderLoop()`（官方要求它在主线程），关窗后通知物理线程收工。时长**默认不限**，关窗才结束；给了 `seconds` 就到那个仿真时刻停止推进物理，窗口继续开着方便观察。

一个坑：`Simulate::Load` 内部会**阻塞等渲染线程来接模型**（条件变量 `cond_loadrequest`），所以顺序必须是「主线程先跑 `RenderLoop()`，再由物理线程 `Load`」（官方 `main.cc` 就是把加载放在 `PhysicsThread` 里）；在 `RenderLoop` 之前调 `Load`，结果是开了一个窗口却一帧不画（任务栏有条目、Alt+Tab 里没有、内容空白）外加永久等待。

### 任务 4 的落点（`cpp/`）

`cpp/` 只放任务 4 的实现：把 [`python/`](python/) 的双缓冲结构用 C++ 复刻（物理线程独占 `mjData`、渲染只读快照副本、锁只罩 memcpy），之后再接官方 `Simulate` 界面或自带渲染循环。目前那里只有一份 `README.md` 占位。

**为什么做 C++**：不是为了更快——`mj_step` 两边调用的是同一份 C 库，单步耗时几乎一样（实测数据见 [`../docs/pitfalls/environment.md`](../docs/pitfalls/environment.md) 的「C++ 工具链」一节），渲染开销也只由 GPU 决定；意义在**工程结构与 sim-to-real**（真实机器人上的控制程序是 C++）。工具链选择（为什么用 pixi 的编译器、编辑器提示怎么配）同样记在那一节。

## 进度

- [ ] 任务 1：认识 MuJoCo（作用、Python 接口、MJCF 结构）
- [x] 任务 2：URDF→MJCF、平地场景、零力矩静止趴卧、力矩执行器（结果见 [`docs/task2.md`](docs/task2.md)；C++ 侧 `cpp_task2/` 同判据）
- [ ] 任务 3：参考 unitree_mujoco 优化代码结构与线程设计（研读笔记 → [`../docs/learn/unitree-mujoco.md`](../docs/learn/unitree-mujoco.md)，线程/通信细节与五种方案的每帧阻滞对比 → [`../docs/learn/runtime-timing.md`](../docs/learn/runtime-timing.md)；**Python 侧已落地**：[`python/`](python/) 用双缓冲把渲染与物理拆开。实测（`scripts/agent_scripts/physics_pacing.py`）：同等 20 ms/次渲染下，无窗口我们 499 步/秒（实时 0.998x）、上游式单锁写法 271 步/秒（0.542x），且物理结果与单线程裸循环逐位相同；开窗口时降到 0.863x——那是 Python 的 GIL 争用（渲染那一步在 Python 里），不是锁，留给任务 4 用 C++ 解决）
- [ ] 任务 4（选做）：用 C++ 重做（任务 2 的 C++ 版已落到 [`cpp_task2/`](cpp_task2/)；任务 4 的双缓冲结构待在 [`cpp/`](cpp/) 实现，对齐 [`python/`](python/)）

任务 3/4 的推进顺序：① C++ 工具链可行性验证（已完成）→ ② 研读 `unitree_mujoco`、写 `docs/learn/unitree-mujoco.md`（已完成）→ ③ Python 侧按新结构重构（**已完成**：`python/`，`scripts/` 里的旧脚本暂留作对照）→ ④ C++ 复刻同一结构（先无窗口 + 录制，再接官方 `Simulate` 界面）。
