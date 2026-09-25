#!/usr/bin/env python3
"""A/B 对照：只差「机器人模型有没有打过基座高度补丁」，看默认位形会不会把狗弹飞。

对每个场景都用**模型默认位形**推进（不加载 keyframe、不覆写 qpos、ctrl=0），
记录初始脚底穿透深度、最大/最终基座高度、最大广义速度。

用法：
    pixi run python scripts/agent_scripts/ab_initial_state.py \
        scenes/flat_scene_raw.xml scenes/flat_scene.xml
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco

# 脚本可能被放在 scripts/ 下任意层级，向上找**同时**含 scenes/ 与 models/ 的目录
ROOT = next(
    p for p in Path(__file__).resolve().parents
    if (p / "scenes").is_dir() and (p / "models").is_dir()
)


def foot_clearance(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """脚底球体下缘的最低世界 z（负值 = 陷入地面多少）。"""
    feet = [
        g
        for g in range(model.ngeom)
        if model.body(model.geom_bodyid[g]).name.endswith("_foot")
        and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
    ]
    return min(data.geom_xpos[g][2] - model.geom_size[g][0] for g in feet)


def run(scene: Path, seconds: float = 2.0) -> dict[str, float]:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)                  # 默认位形，不 resetDataKeyframe

    mujoco.mj_forward(model, data)
    initial_clearance = foot_clearance(model, data)
    initial_z = float(data.qpos[2])

    steps = int(round(seconds / model.opt.timestep))
    max_z, max_qvel = initial_z, 0.0
    for _ in range(steps):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        max_z = max(max_z, float(data.qpos[2]))
        max_qvel = max(max_qvel, float(abs(data.qvel).max()))

    return {
        "初始脚底 z": initial_clearance,
        "初始基座 z": initial_z,
        "2 s 内最高基座 z": max_z,
        "2 s 内最高 |qvel|": max_qvel,
        "最终基座 z": float(data.qpos[2]),
    }


def main(argv: list[str]) -> int:
    scenes = [Path(a) for a in argv[1:]] or [
        ROOT / "scenes/flat_scene_raw.xml",
        ROOT / "scenes/flat_scene.xml",
    ]

    rows = {}
    for scene in scenes:
        path = scene if scene.is_absolute() else (Path.cwd() / scene)
        rows[scene.name] = run(path.resolve())

    keys = list(next(iter(rows.values())))
    width = max(len(n) for n in rows) + 2
    print(f"{'场景':<{width}}" + "".join(f"{k:>18}" for k in keys))
    for name, values in rows.items():
        print(f"{name:<{width}}" + "".join(f"{values[k]:>18.4f}" for k in keys))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
