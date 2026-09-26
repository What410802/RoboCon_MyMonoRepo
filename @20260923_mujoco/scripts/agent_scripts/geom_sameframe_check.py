#!/usr/bin/env python3
"""复现两个「地面斜了，画面却看不出」的陷阱（给 docs/learn/mujoco.md §6.7/§6.8 提供可复现证据）。

A. 运行时改地面 geom 的 pos/quat 会被 `geom_sameframe` 静默吃掉：
   模型字段改了、`d->geom_xpos`/`d->geom_xmat` 与碰撞面却都不动；清掉 sameframe 才生效。
B. 相机不会跟着地面转（自由相机永远重力水平），而且纯色无限地面即使真转了也看不出来：
   不带纹理的地面，pitch 0° 与 15° 的画面在下部条带（只有地面）几乎逐像素相同；
   给地面加棋盘格纹理后差异立刻出来。

用法：
  pixi run python @20260923_mujoco/scripts/agent_scripts/geom_sameframe_check.py [--pitch 15] [--size 240x135]
  （B 需要离屏渲染，仓库默认的 MUJOCO_GL=egl 即可）
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

# 脚本可能在 scripts/ 下任意层级：向上找同时含 scenes/ 与 models/ 的目录（与 rest_check.py 同规则）
ROOT = next(p for p in Path(__file__).resolve().parents
            if (p / "scenes").is_dir() and (p / "models").is_dir())
SLOPE_SCENE = ROOT / "scenes/slope_scene.xml"   # 带棋盘格纹理
FLAT_SCENE = ROOT / "scenes/flat_scene.xml"     # 纯色地面

BALL_XML = """
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
    <body name="ball" pos="1 0 0.5"><freejoint/><geom name="ball" type="sphere" size="0.05"/></body>
  </worldbody>
</mujoco>
"""


def tilt_quat(pitch_deg: float) -> np.ndarray:
    q = np.zeros(4)
    mujoco.mju_euler2Quat(q, np.radians([0.0, pitch_deg, 0.0]), "xyz")
    return q


def floor_normal(data, geom_id: int) -> np.ndarray:
    """地面 geom 在世界系里的实际法向 = geom_xmat 的第三列（引擎真正用的那个）。"""
    return data.geom_xmat.reshape(-1, 3, 3)[geom_id][:, 2]


def part_a(pitch: float) -> None:
    print("== A. 运行时改 geom_quat：碰撞面到底动没动（最小模型：地面 + 自由小球）==")
    for tag, clear_sameframe in (("只改 quat        ", False),
                                 ("quat + 清 sameframe", True)):
        m = mujoco.MjModel.from_xml_string(BALL_XML)
        d = mujoco.MjData(m)
        fid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        sameframe = int(m.geom_sameframe[fid])
        m.geom_quat[fid] = tilt_quat(pitch)
        if clear_sameframe:
            m.geom_sameframe[fid] = 0
        mujoco.mj_forward(m, d)
        while d.time < 2.0:
            mujoco.mj_step(m, d)
        p = d.qpos[:3]
        print(f"  {tag}: geom_sameframe={sameframe} → 地面实测法向 "
              f"({floor_normal(d, fid)[0]:+.3f}, {floor_normal(d, fid)[1]:+.3f}, "
              f"{floor_normal(d, fid)[2]:+.3f})，小球落点 x={p[0]:.3f} z={p[2]:.3f}")
    print(f"  参考：小球半径 0.05；从 x=1 落下，地面若真转 {pitch:g}° 它会沿坡滚走")


def render(model: mujoco.MjModel, pitch: float, clear_sameframe: bool,
           size: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    """按 pivot 把地面转 pitch，渲染一帧；返回 (图像, 相机 pos/forward/up)。"""
    m = mujoco.MjModel.from_xml_path(str(model))
    d = mujoco.MjData(m)
    fid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    mat = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_MATERIAL, "floor_grid_mat")  # flat 场景没有 → -1
    if mat >= 0:
        m.geom_matid[fid] = mat
    if pitch != 0.0:
        m.geom_quat[fid] = tilt_quat(pitch)
        if clear_sameframe:
            m.geom_sameframe[fid] = 0
        # 狗也跟着转（与斜面 demo 一致），这样两张图的区别只剩「地面转没转」
        d.qpos[3:7] = tilt_quat(pitch)
    mujoco.mj_forward(m, d)
    renderer = mujoco.Renderer(m, size[1], size[0])
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(m, cam)
    renderer.update_scene(d, camera=cam)
    scn = renderer.scene
    basis = np.concatenate([scn.camera[0].pos, scn.camera[0].forward, scn.camera[0].up])
    img = renderer.render().astype(int)
    renderer.close()
    return img, basis


def part_b(pitch: float, size: tuple[int, int]) -> None:
    print("\n== B. 相机有没有跟着地面转 / 没有纹理的地面看不看得出倾斜 ==")
    base, base_basis = render(FLAT_SCENE, 0.0, True, size)
    only_quat, tilted_basis = render(FLAT_SCENE, pitch, False, size)
    fixed, _ = render(SLOPE_SCENE, pitch, True, size)
    print(f"  自由相机 pos/forward/up（pitch 0° 与 {pitch:g}° 同一个相机）：")
    print(f"    pitch 0°: {np.round(base_basis, 4).tolist()}")
    print(f"    pitch {pitch:g}°: {np.round(tilted_basis, 4).tolist()}  "
          f"→ up·世界z={base_basis[8]:.5f}（两个方向都逐位相同 = 相机没动）")
    band = slice(int(size[1] * 0.6), None)  # 下部条带：画面里只有地面
    for tag, img in (("旧写法（只改 quat）", only_quat), ("修复后（清 sameframe）", fixed)):
        diff = np.abs(img[band] - base[band]).sum(axis=2)
        print(f"  {tag}：与水平地面基准的下部条带平均 |ΔRGB| = {diff.mean():.2f}"
              f"（上限 765；≈0 = 画面里地面根本没变）")
    print("  注意：这次比较用纯色地面（scenes/flat_scene.xml）。带棋格格的 slope_scene.xml 是为了"
          "看得更清楚（能看出斜多少、狗的立姿相对竖直方向怎么歪），见 docs/learn/mujoco.md §6.8")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pitch", type=float, default=15.0, help="倾斜角度（度，绕 y 轴），默认 15")
    ap.add_argument("--size", default="240x135", help="B 部分渲染尺寸 WxH，默认 240x135")
    args = ap.parse_args()
    w, h = (int(v) for v in args.size.lower().split("x"))

    print(f"mujoco {mujoco.__version__}；pitch={args.pitch:g}°")
    part_a(args.pitch)
    part_b(args.pitch, (w, h))


if __name__ == "__main__":
    main()
