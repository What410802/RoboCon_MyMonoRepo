# RoboCon 培训学习仓库

四足组培训期间的任务工作区。每个任务一个 `@<日期>_<主题>` 目录；Python / MuJoCo / C++ 工具链由仓库根目录的 `pixi.toml` 统一声明，不依赖系统 apt 包。

项目约定（提交信息、文档、目录、Git 用法、环境）见 [`docs/conventions.md`](docs/conventions.md)。

## 任务记录

| 日期 | 任务 | 目录 | 任务文档 |
|---|---|---|---|
| 2026-09-22 | C++ / OOP / CMake 基础 | [`@20260922_robot_cpp_training/robot_cpp_oop_cmake_training/`](@20260922_robot_cpp_training/robot_cpp_oop_cmake_training/) | [`验收.md`](@20260922_robot_cpp_training/验收.md) |
| 2026-09-23 | URDF 转换、MuJoCo 仿真（含 C++ 复刻） | [`@20260923_mujoco/`](@20260923_mujoco/) | [`README.md`](@20260923_mujoco/README.md) |

## 快速开始

```bash
pixi install                                           # 按 pixi.lock 还原环境（Python + MuJoCo + C++ 工具链）
pixi run python @20260923_mujoco/scripts/simulate.py   # Python 侧：平地场景开窗口跑零力矩仿真
```

各任务的完整运行方式（脚本参数、C++ 构建与运行、产物位置）见对应任务文档。

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
