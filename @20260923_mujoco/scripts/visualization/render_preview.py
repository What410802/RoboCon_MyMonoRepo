#!/usr/bin/env python3
"""离屏渲染一张场景截图，用来确认模型外观与初始姿态。

离屏渲染需要一个 OpenGL 上下文，用环境变量 MUJOCO_GL 选择后端：
  * egl    —— 本机有 libEGL_nvidia / libEGL_mesa，无窗口也能渲染（默认走这个）
  * glfw   —— 借用 X 显示（本机 display :1）
  * osmesa —— 纯软件渲染，需要系统装 libosmesa6（当前未装）
PNG 直接用标准库 zlib 手写，避免为一个截图引入 pillow/imageio 依赖。

用法：
    pixi run python scripts/render_preview.py [--camera front|side|iso] [--out output/preview_<camera>.png]
"""

from __future__ import annotations

import argparse
import pathlib
import struct
import sys
import zlib

import mujoco

# 脚本可能被放在 scripts/ 下任意层级，向上找**同时**含 scenes/ 与 models/ 的目录
ROOT = next(
    p for p in pathlib.Path(__file__).resolve().parents
    if (p / "scenes").is_dir() and (p / "models").is_dir()
)
# 注意：本场景自带 <keyframe name="rest">；下面若有 keyframe 就把它当初始位形
# （所以默认截图是“趴卧”姿态，而不是站立位形）。换场景时这个依赖会变。
SCENE = ROOT / "scenes/flat_scene.xml"

CAMERAS = {  # azimuth, elevation, distance, lookat
    "iso": (135.0, -20.0, 2.0, (0.0, 0.0, 0.15)),
    "front": (180.0, -8.0, 1.6, (0.0, 0.0, 0.12)),
    "side": (90.0, -8.0, 1.6, (0.0, 0.0, 0.12)),
    "top": (135.0, -75.0, 1.8, (0.0, 0.0, 0.1)),
}


def write_png(path: pathlib.Path, rgb) -> None:
    """把 HxWx3 的 uint8 数组写成 PNG（只用标准库）。"""
    height, width, _ = rgb.shape

    def chunk(tag: bytes, data: bytes) -> bytes:
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(height))
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", choices=sorted(CAMERAS), default="iso")
    parser.add_argument("--out", type=pathlib.Path, default=None,
                        help="默认 output/preview_<camera>.png")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()
    out = args.out or (ROOT / f"output/preview_{args.camera}.png")

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    azimuth, elevation, distance, lookat = CAMERAS[args.camera]
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.azimuth = azimuth
    camera.elevation = elevation
    camera.distance = distance
    camera.lookat[:] = lookat

    import os

    print(f"MUJOCO_GL={os.environ.get('MUJOCO_GL', '(unset)')}")
    with mujoco.Renderer(model, args.height, args.width) as renderer:
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()

    report_pixels(pixels)
    write_png(out, pixels)
    print(f"渲染 {args.camera} 视角 -> {out}  ({args.width}x{args.height})")
    return 0


def report_pixels(pixels) -> None:
    """粗略判断画面是否真的画出了东西（背景取左上角像素）。"""
    import numpy as np

    background = pixels[0, 0].astype(int)
    mask = np.abs(pixels.astype(int) - background).sum(axis=2) > 12
    lines = [f"  像素统计：mean={pixels.mean():.1f}  非背景占比={mask.mean():.3f}"]
    rows, cols = np.nonzero(mask)
    if rows.size:
        lines.append(f"  非背景包围盒：x[{cols.min()},{cols.max()}] y[{rows.min()},{rows.max()}]")
    else:
        lines.append("  画面除背景外全空，渲染可能失败")
    print("\n".join(lines))


if __name__ == "__main__":
    sys.exit(main())
