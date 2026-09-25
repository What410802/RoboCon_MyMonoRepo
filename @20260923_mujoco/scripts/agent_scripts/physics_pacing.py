#!/usr/bin/env python3
"""任务 3 的检查脚本：新循环要「物理结果不变、但物理不再被渲染顶住」。

四组对照，全部用真实模型与真实 `mj_step`，只把"渲染"这一步换成可控开销：

0. **物理一致性**：单线程裸循环（无锁无线程）跑 N 步，与 `Simulator` 跑同样步数逐项对比 `qpos`，
   要求**完全一致**（确定性物理不该被线程化改变）；
1. **基线**：`Simulator` 无窗口实时跑，渲染开销≈0；
2. **我们的设计**：把 `Simulator._render_once` 换成"搬快照 + mj_forward + 睡 20 ms"
   （20 ms ≈ 本机 `viewer.sync()` 的量级，见 docs/pitfalls/environment.md 的图形后端一节），再看每秒步数；
3. **上游写法**：一把锁同时罩住 `mj_step` 与渲染（上游 Python 版就是这样，见
   docs/learn/unitree-mujoco.md §7 的第 1、2 条），同样 20 ms 渲染。

期望：2 ≈ 1（物理不被渲染顶住），3 明显低于 1（这正是上游要把 `SIMULATE_DT` 放大到 0.005 的原因）。

用法：
    pixi run python @20260923_mujoco/scripts/agent_scripts/physics_pacing.py
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
import types
from pathlib import Path

import mujoco
import numpy as np

# 向上找**同时**含 scenes/ 与 models/ 的目录，再把 python/ 加进导入路径（普通脚本，不走包）
ROOT = next(
    p for p in Path(__file__).resolve().parents
    if (p / "scenes").is_dir() and (p / "models").is_dir()
)
sys.path.insert(0, str(ROOT / "python"))

from control import ZeroTorque  # noqa: E402
from simulator import STATE_FIELDS, Simulator  # noqa: E402

SCENE = ROOT / "scenes/flat_scene.xml"


def load_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(SCENE))


def run_plain(model: mujoco.MjModel, steps: int) -> np.ndarray:
    """单线程裸循环：没有任何锁和线程，作为物理结果的参照物。"""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for _ in range(steps):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
    return data.qpos.copy()


def run_ours(model: mujoco.MjModel, seconds: float, render_ms: float) -> tuple[int, float]:
    """用 Simulator 跑 seconds 秒墙钟；render_ms > 0 时替换成慢渲染。"""
    sim = Simulator(model, ZeroTorque(), realtime=True, use_viewer=False)
    if render_ms > 0:
        sim._render_once = types.MethodType(slow_render(render_ms), sim)
    timer = threading.Timer(seconds, sim.stop)
    started = time.perf_counter()
    timer.start()
    sim.run()
    wall = time.perf_counter() - started
    timer.cancel()
    return sim.steps, wall


def slow_render(render_ms: float):
    """慢渲染版 `_render_once`：搬快照 + mj_forward + 一段固定开销。"""

    def render(self, viewer) -> None:  # noqa: ANN001
        with self._snap_lock:
            for field in STATE_FIELDS:
                np.copyto(getattr(self.render_data, field), self._snap[field])
            self.render_data.time = self._snap_time
        mujoco.mj_forward(self.model, self.render_data)
        time.sleep(render_ms / 1000.0)

    return render


def run_upstream_style(model: mujoco.MjModel, seconds: float, render_ms: float, viewer_dt: float = 0.02) -> tuple[int, float]:
    """上游写法：物理与渲染共用一把锁，锁跨住整个 `mj_step`。"""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    dt = model.opt.timestep
    locker = threading.Lock()
    stop = threading.Event()
    steps = [0]

    def physics() -> None:
        deadline = time.perf_counter()
        while not stop.is_set():
            locker.acquire()          # 要等渲染把锁放开
            data.ctrl[:] = 0.0
            mujoco.mj_step(model, data)
            steps[0] += 1
            locker.release()
            deadline += dt
            delay = deadline - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.perf_counter()

    def viewer() -> None:
        while not stop.is_set():
            locker.acquire()
            time.sleep(render_ms / 1000.0)   # 相当于 viewer.sync()
            locker.release()
            time.sleep(viewer_dt)

    threads = [threading.Thread(target=physics, name="physics"), threading.Thread(target=viewer, name="viewer")]
    started = time.perf_counter()
    for thread in threads:
        thread.start()
    time.sleep(seconds)
    stop.set()
    for thread in threads:
        thread.join()
    return steps[0], time.perf_counter() - started


def main() -> int:
    parser = argparse.ArgumentParser(description="检查渲染开销是否影响物理步进")
    parser.add_argument("--seconds", type=float, default=3.0, help="每组对照的墙钟时长")
    parser.add_argument("--render-ms", type=float, default=20.0, help="模拟的渲染单次开销")
    parser.add_argument("--parity-steps", type=int, default=2000, help="物理一致性对比的步数")
    args = parser.parse_args()

    model = load_model()
    dt = model.opt.timestep
    print(f"model: nq={model.nq} nv={model.nv} nu={model.nu} dt={dt}（每秒 {1 / dt:.0f} 步是实时目标）")

    # 0. 物理一致性：线程化不能改变物理结果（用同一条步数的裸循环逐位对比）
    sim = Simulator(model, ZeroTorque(), realtime=False, use_viewer=False, seconds=args.parity_steps * dt)
    sim.run()
    reference = run_plain(model, sim.steps)
    same = np.array_equal(reference, sim.physics_data.qpos)
    drift = float(np.abs(sim.physics_data.qpos[:2] - model.key_qpos[0, :2]).max())
    print(f"[0] 物理一致性: steps={sim.steps} 与单线程裸循环逐位相同={same} 基座 xy 偏移={drift:.3e} m")

    # 1~3. 渲染开销的对照
    base_steps, base_wall = run_ours(model, args.seconds, render_ms=0.0)
    slow_steps, slow_wall = run_ours(model, args.seconds, render_ms=args.render_ms)
    ups_steps, ups_wall = run_upstream_style(model, args.seconds, render_ms=args.render_ms)
    rows = [
        ("[1] 基线（渲染≈0）", base_steps, base_wall),
        (f"[2] 我们（渲染 {args.render_ms:.0f} ms）", slow_steps, slow_wall),
        (f"[3] 上游式（渲染 {args.render_ms:.0f} ms）", ups_steps, ups_wall),
    ]
    for label, steps, wall in rows:
        print(f"{label}: steps={steps:5d} 步/秒={steps / wall:7.1f} 实时率={steps * dt / wall:.3f}x")

    # 零力矩 4 s 后基座只挪 µm 量级（接触求解器的正常残差）；这里只做粗筛，
    # 严格的"静止趴卧"判据在 scripts/agent_scripts/rest_check.py
    ok = (
        same
        and drift < 1e-4
        and slow_steps >= 0.9 * base_steps
        and ups_steps <= 0.6 * base_steps
    )
    print("结论: " + ("通过（物理不变、且渲染不再顶住物理；上游写法确实被顶住）" if ok else "未通过，见上面数字"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
