#!/usr/bin/env python3
"""双缓冲 + 两条线程的仿真循环（任务 3 的落点）。

与上游 unitree_mujoco 的差别，也就是任务 3 要改的地方（对照 docs/learn/unitree-mujoco.md §7、§8）：

- **物理线程独占 `physics_data`**：只有它读写这份 `mjData`，而且 `mj_step()` 不在任何锁里。
- **渲染用另一份 `render_data`**：主线程只把"最新快照"搬进去再刷窗口，
  所以 `viewer.sync()` 的开销（本机实测 7~20 ms）不会顶住物理步进。
- **锁只出现在两次 memcpy 上**：物理线程写完快照就解锁、渲染线程读完就解锁，
  临界区是微秒级；没有哪把锁跨过 `mj_step`。
- **实时性用 deadline pacing**：按"第 n 步应在何时"的绝对时刻排程，睡眠误差不累积；
  落后太多（被抢占）时重新对齐，不做追赶。

线程只有两条：`physics` + 主线程（渲染线程，`use_viewer=False` 时主线程只按渲染节奏消费快照）。
"""

from __future__ import annotations

import threading
import time

import mujoco
import mujoco.viewer   # launch_passive 在子模块里，必须显式导入
import numpy as np

# 快照要带的字段：qpos/qvel 是运动状态，act 是执行器内部状态，ctrl 供渲染显示
STATE_FIELDS = ("qpos", "qvel", "act", "ctrl")


class Simulator:
    def __init__(self, model, control, *, realtime=True, use_viewer=True, seconds=None, keyframe=0, viewer_fps=60.0):
        self.model = model
        self.control = control
        self.realtime = realtime
        self.use_viewer = use_viewer
        self.seconds = seconds          # 跑这么多仿真秒后请求退出；None = 一直跑
        self.viewer_period = 1.0 / viewer_fps

        self.physics_data = mujoco.MjData(model)   # 物理线程独占
        self.render_data = mujoco.MjData(model)    # 渲染线程独占
        self._snap = {f: np.array(getattr(self.physics_data, f), copy=True) for f in STATE_FIELDS}
        self._snap_time = 0.0
        self._snap_lock = threading.Lock()         # 只保护上面这几个数组的搬运

        self.steps = 0
        self.running = False
        self._physics_thread = None
        self._wall_start = 0.0
        self._wall_seconds = 0.0

        if keyframe is not None:
            mujoco.mj_resetDataKeyframe(model, self.physics_data, keyframe)
            mujoco.mj_resetDataKeyframe(model, self.render_data, keyframe)

    # ---------------------------------------------------------------- 物理线程
    def _physics_loop(self) -> None:
        model, data = self.model, self.physics_data
        dt = model.opt.timestep
        deadline = time.perf_counter()
        while self.running:
            self.control.update(data)
            mujoco.mj_step(model, data)
            with self._snap_lock:                  # 临界区只有 memcpy
                for field in STATE_FIELDS:
                    np.copyto(self._snap[field], getattr(data, field))
                self._snap_time = data.time
            self.steps += 1
            if self.seconds is not None and data.time >= self.seconds:
                self.running = False
            if not self.realtime:
                continue
            deadline += dt
            delay = deadline - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.perf_counter()     # 落后了就直接重新对齐，不追赶

    # ------------------------------------------------------------------ 渲染
    def _render_once(self, viewer) -> None:
        """把最新快照搬进 render_data 并刷一次窗口。

        检查脚本会把本方法整体替换成"慢渲染"，所以这里单独成一个方法。
        """
        with self._snap_lock:
            for field in STATE_FIELDS:
                np.copyto(getattr(self.render_data, field), self._snap[field])
            self.render_data.time = self._snap_time
        mujoco.mj_forward(self.model, self.render_data)   # 补齐 xpos/xquat 等派生量
        if viewer is not None:
            viewer.sync()

    # ------------------------------------------------------------------ 主循环
    def run(self) -> None:
        """主线程：起物理线程 + 跑渲染循环；返回时物理线程已收尾。"""
        self.running = True
        self._wall_start = time.perf_counter()
        self._physics_thread = threading.Thread(target=self._physics_loop, name="physics")
        self._physics_thread.start()

        if self.use_viewer:
            with mujoco.viewer.launch_passive(self.model, self.render_data) as viewer:
                while viewer.is_running() and self.running:
                    self._render_once(viewer)
                    time.sleep(self.viewer_period)
                self.stop()        # 先停物理线程再关窗口：上游那种"关窗时物理还在跑"的竞态就没有了
        else:
            # 无窗口时也照样按渲染节奏消费快照，让检查脚本能量到同一条代码路径
            while self.running:
                self._render_once(None)
                time.sleep(self.viewer_period)
            self.stop()

        self._wall_seconds = time.perf_counter() - self._wall_start

    def stop(self) -> None:
        """请求退出并等物理线程收尾；可以重复调用。"""
        self.running = False
        if self._physics_thread is not None:
            self._physics_thread.join()
            self._physics_thread = None

    # ------------------------------------------------------------------ 结果
    def summary(self) -> str:
        wall = max(self._wall_seconds, 1e-9)
        sim_seconds = self.steps * self.model.opt.timestep
        return (
            f"steps={self.steps} sim={sim_seconds:.3f}s wall={wall:.3f}s "
            f"realtime={sim_seconds / wall:.3f}x step={1000 * wall / max(self.steps, 1):.4f}ms"
        )
