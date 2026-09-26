# RoboCon 培训学习仓库

四足组培训期间的任务工作区。每个任务一个 `@<日期>_<主题>` 目录；Python / MuJoCo / C++ 工具链由仓库根目录的 `pixi.toml` 统一声明，不依赖系统 apt 包。

项目约定（提交信息、文档、目录、Git 用法、环境）见 [`docs/conventions.md`](docs/conventions.md)。

## 任务记录

| 日期 | 任务 | 目录 | 任务文档 |
|---|---|---|---|
| 2026-09-22 | C++ / OOP / CMake 基础 | [`@20260922_robot_cpp_training/robot_cpp_oop_cmake_training/`](@20260922_robot_cpp_training/robot_cpp_oop_cmake_training/) | [`验收.md`](@20260922_robot_cpp_training/验收.md) |
| 2026-09-23 | URDF 转换、MuJoCo 仿真（含 C++ 复刻） | [`@20260923_mujoco/`](@20260923_mujoco/) | [`README.md`](@20260923_mujoco/README.md) |

## 快速开始

环境只有**一个**（本仓库根目录的 `pixi.toml` + `pixi.lock`），装它分两步：先装 `pixi` 本身（单文件包管理器，默认装到 `~/.pixi`，不需要 sudo；官方说明见 <https://pixi.prefix.dev/latest/installation/>），再用它还原环境。

```bash
# ① 装 pixi（装完重开终端或 source 一下 shell 配置以更新 PATH；以后升级用 pixi self-update）
curl -fsSL https://pixi.sh/install.sh | sh

# ② 在仓库根目录还原环境：Python + MuJoCo（含 C++ 头文件/库/CMake 配置）+ C++ 工具链
cd <本仓库根目录>            # 有 pixi.toml 的地方
pixi install

# ③ 验证：版本号，以及 C++ 侧要用的头文件与 CMake 配置都在
pixi run python -c "import mujoco; print(mujoco.__version__)"                        # 期望 3.12.0
pixi run bash -lc 'ls "$CONDA_PREFIX/include/mujoco/mujoco.h" "$CONDA_PREFIX/lib/cmake/mujoco"'

# ④ 跑一个例子：平地场景零力矩仿真（开窗口，需显示服务；仓库默认 egl 无窗口）
pixi run env MUJOCO_GL=glfw python @20260923_mujoco/scripts/simulate.py
```

各任务的完整运行方式（脚本参数、C++ 构建与运行、产物位置）见对应任务文档；为什么这么搭环境、踩过哪些坑见 [`docs/pitfalls/environment.md`](docs/pitfalls/environment.md)。

## 文档索引

`docs/` 下分两类：`learn/`（研究与学习）、`pitfalls/`（踩坑记录）；项目约定单独一份放 `docs/conventions.md`。

**研究与学习（`docs/learn/`）**

| 文档 | 内容 |
|---|---|
| [`mujoco.md`](docs/learn/mujoco.md) | MuJoCo 知识点与坑点：`MjModel` / `MjData`、`geom` / `friction` / `condim`、渲染后端（§7）、决策反向索引（[§7.6](docs/learn/mujoco.md#76-这些结论驱动了哪些配置决策)）、复现命令（§8） |
| [`graphics-stack.md`](docs/learn/graphics-stack.md) | 图形 / 渲染 / 视频栈速查（OpenGL、Skia、DirectX、EGL、GLFW 各在哪一层） |
| [`unitree-mujoco.md`](docs/learn/unitree-mujoco.md) | 上游 `unitree_mujoco` 研读笔记：架构、线程、通信与目标设计 |
| [`unitree-mujoco-threads.md`](docs/learn/unitree-mujoco-threads.md) | 同上，展开到进程 / 线程 / 通信的时序细节 |
| [`cpp-cmake.md`](docs/learn/cpp-cmake.md) | C++ 与 CMake 问答笔记（`virtual` / `explicit` / `override`、`const` 成员函数、头文件扩展名、类内 vs 类外定义、CMake target） |
| [`cmake-intellisense.md`](docs/learn/cmake-intellisense.md) | VSCode C++ / CMake 智能提示配置（语言模式、clangd / cpptools、编译数据库） |

**踩坑记录（`docs/pitfalls/`）**

| 文档 | 内容 |
|---|---|
| [`environment.md`](docs/pitfalls/environment.md) | 环境与踩坑记录（本机实测）：为什么用 pixi、conda / PyPI 镜像、URDF→MJCF 与 git 索引、图形后端与显卡、C++ 工具链与编辑器提示 |

**项目约定**

| 文档 | 内容 |
|---|---|
| [`conventions.md`](docs/conventions.md) | 提交信息、文档、目录与命名、环境、验证、外部代码与许可 |

`docs/` 与代码、配置之间是双向链接的：**结论写在文档里**（带实测数据与可复现命令），代码与配置的注释**回指结论**。例：`pixi.toml` 里 `MUJOCO_GL` 的注释指向 [`docs/learn/mujoco.md` §7.6](docs/learn/mujoco.md#76-这些结论驱动了哪些配置决策)。
