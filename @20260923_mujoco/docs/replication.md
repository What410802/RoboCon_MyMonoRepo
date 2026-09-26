# 复现上游参考实现（任务 3 的前置验证）

为了把「`LowCmd` 回调到底跑在哪个线程」从“读代码推断”变成运行期事实，也为了先把参考实现的**预期效果**摸清、给 Python 重构留对照基准，2026-09-25 在本机把上游 `unitree_mujoco` 的 `simulate_python/`、`simulate/`（C++）连同 `example/{python,cpp}/stand_go2.*` 都跑通了一遍。

两个复现环境**不在本仓库内**，放在工作区同级的 `Replicate.d/`（相对本目录是 `../../../Replicate.d/`）：参考克隆仍是 `ReadOnly.d/unitree_mujoco`，只往里加了一个指向官方 MuJoCo 包的软链接，**源代码未改**。搭建过程与全部踩坑（Python 必须是 3.10、`cyclonedds==0.10.2` 只有 cp310 轮子、C++ 侧为何必须另下官方 MuJoCo 包、需要 `libgl-devel` 与 `eigen`、上游按可执行文件位置找配置与场景、退出时段错误）连完整命令一起记在 [`../../docs/pitfalls/environment.md`](../../docs/pitfalls/environment.md) 的「复现上游 unitree_mujoco」一节。

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

两个数不一致本身就是发现：两边时间步不同（Python `SIMULATE_DT = 0.005`、C++ `0.002`），伺服收敛位形因此不同；做 A/B 对比时不能直接把两边的绝对高度拿来比。

复现顺带确认的两件事（对做任务 3 有用）：**回调归属**（带队列的订阅回调跑在 SDK 自建线程 `rlsnr` / `ch_reader`，`queueLen=0` 才跑在 DDS 接收线程）见 [`../../docs/learn/unitree-mujoco-threads.md`](../../docs/learn/unitree-mujoco-threads.md) §10；上游**控制器没有 stdin 控制**、`#define private public` 覆盖 GLFW 回调导致官方快捷键失效等一批问题，见 [`../../docs/learn/unitree-mujoco.md`](../../docs/learn/unitree-mujoco.md) §7 的问题清单。
