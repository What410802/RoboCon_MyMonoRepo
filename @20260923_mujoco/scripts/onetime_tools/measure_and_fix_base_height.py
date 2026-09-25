#!/usr/bin/env python3
"""一次性工具：**量出**「脚底刚好触地」所需的基座高度，并把导出模型**挪**到该高度。

产物：`models/black_description.xml`（网站原始导出 + 这一处修改）。放在
`scripts/onetime_tools/` 是因为它只服务这一台机器人、只跑一次；但它可复现，
将来重新导出后仍能一键重放（也顺便避免手工謄抄 175 行 inertial/quat 数值时出错）。

用法（在仓库根目录执行）：
    pixi run python @20260923_mujoco/scripts/onetime_tools/measure_and_fix_base_height.py

来源
----
`assets/black_description/black_description.xml`
（urdf.enkeebot.com 导出，选项：Floating Base **ON** / Actuator Type **Torque** /
Include Skeleton **OFF** / Mesh Directory `meshes/` / Mesh Format Auto / STL 质量 Original）

补丁清单（只留“不做就出问题”的，不含纯防御性改动）
--------------------------------------------------
1. 基座默认高度：`<body name="trunk" pos="0 0 0">` → `pos="0 0 <触地高度>"`
   导出把基座放在原点，而零位形下脚底在基座下方约 0.5786 m ⇒ **默认状态整只狗
   沉进地面 0.58 m**，任何“建完 MjData 直接 mj_step”的脚本（例如不载 keyframe 的
   最简 viewer 程序）都会看到求解器把狗暴力弹飞。把基座抬到“脚底刚好触地”后，
   默认状态就是四脚站在地面上；零力矩下它会自然塌成趴卧姿态。
   触地高度**不是硬编码常数**，是本脚本用脚底球体几何实时算出来的，
   并在最后重新加载产物校验“默认状态确实没穿模”。
2. 在 `<mujoco>` 后插入来源与补丁说明注释。

刻意不做的改动（记在这里，避免下次又“顺手”加上）
--------------------------------------------------
* **不再改 `meshdir`**。导出自带 `meshdir="meshes/"`，网格重复问题改用**目录软链接**
  解决（真实 STL 只在 `assets/urdf/meshes` 存一份）：

      assets/black_description/meshes -> ../urdf/meshes
      models/meshes                   -> ../assets/urdf/meshes

  这样导出文件里的 `meshdir` 保持原样（git 里只多几十字节的软链接），
  本脚本会顺手把这两个链接建好（缺失就补，冲突就报错）。
* `inertiafromgeom="auto"` **保持导出原样**。它只在 body 没有显式 `<inertial>` 时
  才用 geom 推惯量，而本模型 20 个 body 全都有 `<inertial>`（总质量 13.2472 kg 与
  URDF 一致）⇒ 这个选项当前**完全不起作用**。既然不会有谁给 geom 加 mass/density，
  改成 `false` 就是纯防御性改动，只会让补丁清单变长、diff 变脏，因此不改。
* 不动 `<freejoint/>`、不动 `<motor>` 的 `gear="1" ctrlrange="-20 20"`
  （已与 URDF 的 effort=20 一致）、不动几何与材质。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import mujoco

# 脚本可能被放在 scripts/ 下任意层级，向上找**同时**含 scenes/ 与 models/ 的目录
ROOT = next(
    p for p in Path(__file__).resolve().parents
    if (p / "scenes").is_dir() and (p / "models").is_dir()
)
SRC = ROOT / "assets/black_description/black_description.xml"
DST = ROOT / "models/black_description.xml"

TRUNK_OLD = '<body name="trunk" pos="0 0 0">'

# 目录软链接：真实 mesh 只在 assets/urdf/meshes 存一份，每个**会用到 meshdir="meshes/" 的
# 目录**各自链接过去，这样所有 XML 里的 meshdir 都能保持原样。
#
# 为什么 scenes/ 也要一个：`meshdir` 的解析规则是「相对顶层（main）文件所在目录」
# 而不是相对它自己 —— 结论与复现见 docs/mujoco-notes.md §6.1。所以 scenes/flat_scene.xml
# 去 include models/ 或 assets/ 里的模型时，meshes/ 会在 scenes/ 下找。
MESH_LINKS = {
    ROOT / "assets/black_description/meshes": "../urdf/meshes",
    ROOT / "models/meshes": "../assets/urdf/meshes",
    ROOT / "scenes/meshes": "../assets/urdf/meshes",
}

HEADER = """  <!--
    由 scripts/onetime_tools/measure_and_fix_base_height.py 从
    assets/black_description/black_description.xml 整理而来。
    原始来源：assets/urdf/black_description.urdf
              （N-W-wolf/Training_Materials@main: 第二次培训/black/black_description.urdf，
               sha256 e337a9d619154b98c0484bf4c41e20dd44b48c877d64916a16cb664b4cd12f26）
    导出方式：urdf.enkeebot.com，Floating Base ON / Actuator Type Torque / Include Skeleton OFF
    相对导出只改了一处（见脚本 docstring）：基座默认高度抬到“零位形下脚底刚好触地”，
    否则默认状态会整只狗沉入地面而被弹飞。
    执行器为 <motor>（力矩模式），ctrl 即关节力矩，限幅 ±20 N·m。
    初始趴卧位形另见 scenes/flat_scene.xml 的 <keyframe name="rest">。
  -->
"""


def ensure_mesh_links() -> None:
    """按上面的表把两个 mesh 目录软链接建好（已存在则校验指向）。"""
    for link, target in MESH_LINKS.items():
        if link.is_symlink():
            if os.readlink(link) != target:
                raise SystemExit(f"{link.relative_to(ROOT)} 不是指向 {target} 的软链接")
            continue
        if link.exists():
            raise SystemExit(f"{link.relative_to(ROOT)} 是真实目录（应为软链接），"
                             "请确认它是否又是一份重复的 mesh")
        link.parent.mkdir(parents=True, exist_ok=True)
        link.symlink_to(target)
        print(f"创建软链接 {link.relative_to(ROOT)} -> {target}")


def lowest_foot_z(model: mujoco.MjModel) -> float:
    """模型**默认位形**下，最低的脚底球体下缘的世界 z（负值 = 陷入地面）。"""
    data = mujoco.MjData(model)          # 默认 qpos 即模型默认位形
    mujoco.mj_forward(model, data)
    feet = [
        g
        for g in range(model.ngeom)
        if model.body(model.geom_bodyid[g]).name.endswith("_foot")
        and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
    ]
    if not feet:
        raise SystemExit("没找到 *_foot 球体 geom，无法计算触地高度")
    return min(data.geom_xpos[g][2] - model.geom_size[g][0] for g in feet)


def patch(text: str, base_z: float) -> str:
    trunk_new = f'<body name="trunk" pos="0 0 {base_z:.6f}">'
    if TRUNK_OLD not in text:
        raise SystemExit(f"未找到待替换内容，导出文件可能已变化：{TRUNK_OLD}")
    text = text.replace(TRUNK_OLD, trunk_new, 1)

    marker = "<worldbody>"
    if marker not in text:
        raise SystemExit("未找到 <worldbody>")
    return text.replace(marker, HEADER + marker, 1)


def main() -> int:
    ensure_mesh_links()

    # 先用导出文件算出触地高度（导出里的 meshdir="meshes/" 经软链接能正常加载）
    src_model = mujoco.MjModel.from_xml_path(str(SRC))
    base_z = -lowest_foot_z(src_model)
    print(f"零位形下脚底相对基座 = {-base_z:.6f} m  ->  基座默认高度取 {base_z:.6f} m")

    DST.parent.mkdir(parents=True, exist_ok=True)
    DST.write_text(patch(SRC.read_text(encoding="utf-8"), base_z), encoding="utf-8")

    model = mujoco.MjModel.from_xml_path(str(DST))
    free = sum(1 for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE)
    actual = {
        "nbody": model.nbody,
        "freejoint": free,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "ngeom": model.ngeom,
        "nmesh": model.nmesh,
    }
    mass = float(model.body_mass.sum())

    print(f"wrote {DST.relative_to(ROOT)}")
    print("  " + " ".join(f"{k}={v}" for k, v in actual.items()) + f" mass={mass:.4f} kg")

    expect = {"nbody": 20, "freejoint": 1, "nq": 19, "nv": 18, "nu": 12, "ngeom": 49, "nmesh": 14}
    bad = {k: (v, actual[k]) for k, v in expect.items() if actual[k] != v}
    if bad:
        print(f"!! 与预期不符：{bad}", file=sys.stderr)
        return 1
    if abs(mass - 13.2472) > 1e-3:
        print(f"!! 总质量 {mass:.4f} 与预期 13.2472 kg 不符", file=sys.stderr)
        return 1

    # 关键回归项：默认状态不能穿模，否则求解器会在第一步把狗弹飞
    clearance = lowest_foot_z(model)
    print(f"默认状态脚底下缘 z = {clearance:+.6f} m（应 >= 0）")
    if clearance < -1e-6:
        print("!! 默认状态仍然穿模", file=sys.stderr)
        return 1

    print("  OK：结构、执行器数量、总质量、默认无穿模 均符合预期")
    return 0


if __name__ == "__main__":
    sys.exit(main())
