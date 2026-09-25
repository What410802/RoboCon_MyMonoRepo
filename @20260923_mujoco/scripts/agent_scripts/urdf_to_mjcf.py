#!/usr/bin/env python3
"""用 MuJoCo 自带能力把 URDF 转成 MJCF。

MuJoCo 内部本来就实现了 URDF -> MJCF 的转换，因此不需要第三方工具：

* 方式 A ``--method saveLastXML``：先让 MuJoCo 编译 URDF，再把「最后一次编译所用的
  XML」落盘。对 URDF 输入而言，这份 XML 就是 MuJoCo 生成的等价 MJCF。
* 方式 B ``--method spec``：用 3.2+ 的模型编辑 API，``MjSpec.from_file(urdf)`` 得到
  spec，再用 ``spec.to_xml()`` 序列化成 MJCF（可继续在 Python 里编辑）。

两种方式得到的都是「可被 MuJoCo 直接编译的 MJCF」，但都不包含 ``<actuator>``：
URDF 里没有执行器概念，力矩电机需要我们自己补。
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import mujoco


def convert(src: pathlib.Path, dst: pathlib.Path, method: str) -> None:
    if method == "spec":
        spec = mujoco.MjSpec.from_file(str(src))
        dst.write_text(spec.to_xml(), encoding="utf-8")
        return

    model = mujoco.MjModel.from_xml_path(str(src))
    mujoco.mj_saveLastXML(str(dst), model)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("src", type=pathlib.Path, help="输入的 .urdf")
    parser.add_argument("dst", type=pathlib.Path, help="输出的 .xml (MJCF)")
    parser.add_argument(
        "--method",
        choices=("saveLastXML", "spec"),
        default="saveLastXML",
        help="转换方式，见模块 docstring",
    )
    args = parser.parse_args()

    convert(args.src, args.dst, args.method)

    model = mujoco.MjModel.from_xml_path(str(args.dst))
    print(f"mujoco {mujoco.__version__} [{args.method}] {args.src} -> {args.dst}")
    print(f"  {args.dst.stat().st_size} bytes, nbody={model.nbody} nq={model.nq} nv={model.nv} nu={model.nu}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
