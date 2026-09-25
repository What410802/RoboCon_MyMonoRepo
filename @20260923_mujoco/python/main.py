#!/usr/bin/env python3
"""任务 3 的仿真程序入口：双缓冲 + 两线程，控制目前是零力矩。

用法：
    pixi run python @20260923_mujoco/python/main.py                             # 开窗口（需要 glfw，见下）
    pixi run env MUJOCO_GL=glfw python @20260923_mujoco/python/main.py          # 仓库默认是 egl，开窗口要临时换回 glfw
    pixi run python @20260923_mujoco/python/main.py --no-viewer --seconds 8     # 无窗口跑 8 仿真秒（看吞吐/取数）
    pixi run python @20260923_mujoco/python/main.py --fast                      # 不按真实时间节流，全速跑
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco

from control import ZeroTorque
from simulator import Simulator

# 脚本换个位置也能跑：向上找**同时**含 scenes/ 与 models/ 的目录
ROOT = next(
    p for p in Path(__file__).resolve().parents
    if (p / "scenes").is_dir() and (p / "models").is_dir()
)
SCENE = ROOT / "scenes/flat_scene.xml"


def main() -> int:
    parser = argparse.ArgumentParser(description="双缓冲仿真循环（任务 3）")
    parser.add_argument("--scene", default=str(SCENE), help="场景 xml（默认平地场景）")
    parser.add_argument("--seconds", type=float, default=None, help="跑这么多仿真秒后退出；默认一直跑")
    parser.add_argument("--fast", action="store_true", help="不按真实时间节流，全速跑")
    parser.add_argument("--no-viewer", action="store_true", help="不开窗口")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.scene)
    sim = Simulator(
        model,
        ZeroTorque(),
        realtime=not args.fast,
        use_viewer=not args.no_viewer,
        seconds=args.seconds,
    )
    print(f"model: nq={model.nq} nv={model.nv} nu={model.nu} dt={model.opt.timestep} control={sim.control.name}")

    try:
        sim.run()
    except KeyboardInterrupt:
        sim.stop()
    print(sim.summary())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
