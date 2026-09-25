#!/usr/bin/env python3
"""任务 2 的验证脚本：平地 + 零力矩，狗能不能静止趴住。

做三件事：
  1. 把 12 个关节角置 0、基座姿态置单位四元数；
  2. 令 ctrl 全 0 推进若干秒，统计基座漂移、末段最大广义速度、接触点数。

两种模式：
  --mode drop（找姿态）：关节角置 0、基座姿态置单位四元数，基座高度**由脚底球体几何反算**
      （基座 z = -min(脚球心 z - 脚球半径)，不是拍脑袋写常数），然后自由落下塌成趴卧姿态，
      打印收敛后的 qpos，供粘进 scenes/flat_scene.xml 的 <keyframe>。
  --mode keyframe（验证，默认）：用场景里的 keyframe（或 --qpos）作为初始状态，
      零力矩推进，要求**从第 0 秒起就基本不动**（漂移与末段速度都接近 0）。

用法：
    pixi run python scripts/rest_check.py                  # 验证 keyframe 是否静止
    pixi run python scripts/rest_check.py --mode drop      # 重新求趴卧姿态
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco

# 脚本可能被放在 scripts/ 下任意层级，向上找**同时**含 scenes/ 与 models/ 的目录
ROOT = next(
    p for p in Path(__file__).resolve().parents
    if (p / "scenes").is_dir() and (p / "models").is_dir()
)
# 注意：本场景自带 <keyframe name="rest">，`--mode keyframe`（默认）**依赖它**；
# 换场景/换模型时先确认它存在（脚本会检查 model.nkey 并报错），否则先跑 --mode drop 求一个。
SCENE = ROOT / "scenes/flat_scene.xml"


def foot_geom_ids(model: mujoco.MjModel) -> list[int]:
    """找出所有名字以 _foot 结尾的 body 下的 geom（本模型脚是球体）。"""
    feet = {b for b in range(model.nbody) if model.body(b).name.endswith("_foot")}
    return [
        g
        for g in range(model.ngeom)
        if model.geom_bodyid[g] in feet and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=8.0, help="仿真时长（仿真时间，不是墙钟）")
    parser.add_argument("--mode", choices=("keyframe", "drop"), default="keyframe")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    if args.mode == "drop":
        # --- 关节角 0、基座不旋转，从「脚刚好触地」处自由落下 ---
        data.qpos[:] = 0.0
        data.qpos[3] = 1.0  # 单位四元数 (w x y z)
        mujoco.mj_forward(model, data)

        feet = foot_geom_ids(model)
        if not feet:
            print("!! 没找到 *_foot 球体 geom", file=sys.stderr)
            return 1
        lowest = min(data.geom_xpos[g][2] - model.geom_size[g][0] for g in feet)
        data.qpos[2] = -lowest                        # 抬高到最低点刚好在 z=0
        mujoco.mj_forward(model, data)
        print(f"脚底最低点相对基座 = {lowest:.6f} m  ->  基座 z = {data.qpos[2]:.4f} m")
    else:
        if model.nkey == 0:
            print("!! 场景里没有 keyframe，请先用 --mode drop 求一个", file=sys.stderr)
            return 1
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        print(f"初始（keyframe {model.key(0).name}）：基座 z={data.qpos[2]:.4f}")

    print(f"初始接触点数 ncon={data.ncon}")

    # --- 零力矩推进 ---
    nstep = int(round(args.seconds / model.opt.timestep))
    tail = int(round(1.0 / model.opt.timestep))       # 末段 1 s 用来判断是否已静止

    start_xy = data.qpos[:2].copy()
    max_qvel = 0.0
    tail_start_xy = None

    for step in range(nstep):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        if step == nstep - tail:
            tail_start_xy = data.qpos[:2].copy()
        if step >= nstep - tail:
            max_qvel = max(max_qvel, float(abs(data.qvel).max()))

    total_drift = float(((data.qpos[:2] - start_xy) ** 2).sum() ** 0.5)
    tail_drift = float(((data.qpos[:2] - tail_start_xy) ** 2).sum() ** 0.5)
    print(
        f"{args.seconds:.1f} s 后：基座 z={data.qpos[2]:.4f}  "
        f"总漂移={total_drift:.4f} m  末 1 s 漂移={tail_drift:.3e} m  "
        f"末段 max|qvel|={max_qvel:.3e}  ncon={data.ncon}"
    )

    still = max_qvel < 1e-3 and tail_drift < 1e-3
    print("判定：" + ("静止趴住 ✓" if still else "未静止 ✗（末段仍在运动）"))

    if args.mode == "drop":
        qpos = " ".join(f"{v:.4f}" for v in data.qpos)
        print("\n把收敛姿态粘进 scenes/flat_scene.xml 的 <keyframe>（xy 清零，平面平移等价）：")
        body = qpos.split()
        body[0] = body[1] = "0"
        print(f'    <key name="rest" qpos="{" ".join(body)}"/>')
    return 0 if still else 2


if __name__ == "__main__":
    sys.exit(main())
