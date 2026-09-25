#!/usr/bin/env python3
"""量渲染 / 录像的单次开销，给仓库根 `docs/learn/mujoco.md` 第 7 节提供可复现数据。

各调用的**量级**（`mj_step` 微秒级 ≪ 编码+管道毫秒级 < 渲染毫秒级 ≪ `viewer.sync` 十几毫秒）
与"为什么大头是渲染而不是编码"，见 `docs/learn/mujoco.md` 第 7.2 节；
那台开发机的具体数值也在那里（属于机器相关的结论），本脚本只负责把**当前这台机器**的
实际值打出来。

两个坑：
  * **不同 `MUJOCO_GL` 后端可能落在不同 GPU 上**（双显卡机器尤其明显）
    → 本脚本会先打印 `GL_RENDERER`，比性能前先确认是谁在画；
  * `render()` 之前**必须先 `update_scene(data)`**，否则画的是**空场景**，测出来的成本
    会偏小好几倍（本项目早期就踩过这个坑）。

用法（在仓库根目录执行）：

    pixi run python scripts/agent_scripts/render_cost.py            # 渲染 + 编码
    pixi run python scripts/agent_scripts/render_cost.py --viewer   # 追加量 viewer.sync()（会弹窗）
    pixi run env MUJOCO_GL=glfw python scripts/agent_scripts/render_cost.py

注意 `MUJOCO_GL` 必须在 `import mujoco` 之前生效；pixi 的 `[activation.env]` 会覆盖命令行前缀
里的值，所以换后端要用 `pixi run env MUJOCO_GL=... python ...` 这种写法。
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

import mujoco
import mujoco.gl_context
import mujoco.viewer

# 脚本可能被放在 scripts/ 下任意层级，向上找**同时**含 scenes/ 与 models/ 的目录
ROOT = next(
    p for p in pathlib.Path(__file__).resolve().parents
    if (p / "scenes").is_dir() and (p / "models").is_dir()
)


def best_of(fn, n: int, repeat: int = 3) -> float:
    """跑 repeat 轮、各 n 次，返回最快一轮的平均毫秒数。"""
    best = float("inf")
    for _ in range(repeat):
        t0 = time.perf_counter()
        for _ in range(n):
            fn()
        best = min(best, (time.perf_counter() - t0) / n * 1e3)
    return best


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=pathlib.Path, default=ROOT / "scenes/flat_scene.xml")
    parser.add_argument("--n", type=int, default=100, help="每轮测量次数")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=float, default=50.0, help="录像帧率（只影响编码背压与节流）")
    parser.add_argument("--viewer", action="store_true", help="追加测 viewer.sync()（会弹出窗口）")
    parser.add_argument("--out", type=pathlib.Path, default=pathlib.Path("/tmp/render_cost.mp4"))
    args = parser.parse_args()

    print(f"MuJoCo GL 后端 = {mujoco.gl_context.GLContext.__module__.rsplit('.', 1)[0]}")

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, args.height, args.width)
    try:
        # 先报一下这块 GL context 到底在哪块 GPU 上（Optimus 笔记本上 egl/glfw 可能不是同一块！）
        try:
            from OpenGL import GL
            print(f"  显卡      = {GL.glGetString(GL.GL_RENDERER).decode()}"
                  f"  [{GL.glGetString(GL.GL_VENDOR).decode()}]")
            print(f"  最大纹理  = {GL.glGetIntegerv(GL.GL_MAX_TEXTURE_SIZE)}"
                  f"   GL_VERSION = {GL.glGetString(GL.GL_VERSION).decode()}")
        except Exception as exc:                             # noqa: BLE001
            print(f"  （GL 信息查询失败：{type(exc).__name__}: {exc}）")

        # 注意：必须先 update_scene，否则 render() 画的是**空场景**（快好几倍，测出来没有意义）
        for _ in range(5):                                   # 预热
            renderer.update_scene(data)
            frame = renderer.render()
        t_scene = best_of(lambda: renderer.update_scene(data), args.n)
        t_render = best_of(renderer.render, args.n)
        t_bytes = best_of(frame.tobytes, args.n)
        t_frame = t_scene + t_render + t_bytes
        print(f"{args.width}x{args.height} update_scene        = {t_scene:6.2f} ms/帧")
        print(f"{args.width}x{args.height} render+readback     = {t_render:6.2f} ms/帧")
        print(f"{args.width}x{args.height} tobytes             = {t_bytes:6.2f} ms/帧")
        print(f"{args.width}x{args.height} 合计“出一帧”        = {t_frame:6.2f} ms/帧")
    finally:
        renderer.close()                                     # 不 close 会在退出时报 OpenGL 错

    print(f"mj_step()                                   = "
          f"{best_of(lambda: mujoco.mj_step(model, data), args.n * 10):6.2f} ms/步")

    sys.path.insert(0, str(ROOT / "scripts"))
    from visualization import VideoRecorder                  # noqa: E402

    with VideoRecorder(model, args.out, fps=args.fps, width=args.width,
                       height=args.height, quiet=True) as rec:
        t0 = time.perf_counter()
        for _ in range(args.n):
            rec.capture(data, force=True)                    # 渲染 + 送 ffmpeg（含 x264 背压）
        ms = (time.perf_counter() - t0) / args.n * 1e3
    print(f"capture(force) 全链路                      = {ms:6.2f} ms/帧"
          f"  ⇒ {args.fps:g} fps 约占 {ms * args.fps / 10:.0f}% CPU 时间")
    print(f"  其中：出帧 {t_frame:.2f} ms + 管道/x264 编码 ≈ {ms - t_frame:.2f} ms")

    if args.viewer:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print(f"viewer.sync()                              = "
                  f"{best_of(viewer.sync, args.n):6.2f} ms/次")
    return 0


if __name__ == "__main__":
    sys.exit(main())
