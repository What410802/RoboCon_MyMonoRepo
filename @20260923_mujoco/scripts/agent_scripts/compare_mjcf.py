#!/usr/bin/env python3
"""对比多个 MuJoCo 模型文件（URDF 或 MJCF）的结构差异。

用法：
    pixi run python scripts/agent_scripts/compare_mjcf.py FILE...
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco


def inspect(path: Path) -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(str(path))
    total_mass = float(model.body_mass.sum())

    # 模型里是否定义了自由基座（决定机器人能不能整体平动/转动）
    freejoints = sum(1 for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE)
    # 执行器传动类型种类数（0 表示完全没有执行器）
    act_types = len(set(model.actuator_trntype.tolist()))

    return {
        "file": path.name,
        "size": path.stat().st_size,
        "nbody": model.nbody,
        "njnt": model.njnt,
        "free": freejoints,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "ngeom": model.ngeom,
        "nmesh": model.nmesh,
        "nmat": model.nmat,
        "mass": total_mass,
        "act": act_types,
    }


def main(argv: list[str]) -> int:
    paths = [Path(a) for a in argv[1:]]
    rows = []
    for path in paths:
        try:
            rows.append(inspect(path))
        except Exception as exc:  # noqa: BLE001 - 对比脚本，报错继续
            print(f"!! {path.name}: {type(exc).__name__}: {exc}", file=sys.stderr)

    if not rows:
        return 1

    cols = list(rows[0])
    header = {c: c for c in cols}
    header["size"] = "size(B)"
    header["mass"] = "mass(kg)"
    header["free"] = "freejnt"
    header["act"] = "actTypes"
    header["nmat"] = "nmat"
    names = [str(r["file"]) for r in rows]
    width = max(len(n) for n in names) + 2

    print(f"{'file':<{width}}" + "".join(f"{header[c]:>10}" for c in cols[1:]))
    for row in rows:
        line = f"{str(row['file']):<{width}}"
        for c in cols[1:]:
            value = row[c]
            line += f"{value:>10}" if isinstance(value, int) else f"{value:>10.4f}" if isinstance(value, float) else f"{value!s:>10}"
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
