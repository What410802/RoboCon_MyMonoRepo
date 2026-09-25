#!/usr/bin/env python3
"""示例：把录像接进**你自己的**仿真脚本 —— 循环一行都不用重写，也**不需要窗口**。

它同时是本仓库**唯一**的“零力矩录像”命令行入口：不写代码，给个场景就能录。

起因：最早的录像脚本（`record_video.py`）自己带了一个仿真循环，想录自己的控制逻辑，
就只能把逻辑抄进那个脚本里。本文件演示解耦后的做法：
**循环还在你手里**，录像只是一个 3 行的插件。

    你的脚本                                 接入后（新增只有 ①②③）
    ---------------------------------------  ----------------------------------------
                                             from visualization import VideoRecorder  # ①
    model = mujoco.MjModel.from_xml_path(..) model = mujoco.MjModel.from_xml_path(..)
    data  = mujoco.MjData(model)             data  = mujoco.MjData(model)
                                             with VideoRecorder(model,"out.mp4") as rec:  # ②
    while 你的条件:                             while 你的条件:
        你的控制逻辑                                你的控制逻辑
        mujoco.mj_step(model, data)               mujoco.Mj_step(model, data)
                                                  rec.capture(data)                  # ③

不用你管的事：相机、离屏渲染、帧率、ffmpeg 编码、文件收尾 —— 全在库里。

* **不用开窗口**：离屏渲染走 EGL（仓库根 `pixi.toml` 的 `[activation.env]` 已设 `MUJOCO_GL=egl`）。
  想连同窗一起看才需要 `launch_passive`，见 `example_with_viewer.py`。
* ① 的导入路径：`scripts/` 下的脚本直接 `from visualization import VideoRecorder` 就行
  （`visualization/__init__.py` 把常用名字提到包一级了）；本文件自己在 `examples/` 里，
  所以开头先把 `scripts/` 加进 `sys.path`。
* 每个仿真步调一次 `capture(data)`；内部按**仿真时间** `data.time` 判断该不该出帧，
  所以输出 MP4 的时间轴 = 仿真时间（改帧率只改 `fps=`，不用动循环），
  **与渲染耗时/机器快慢无关**（慢机器只是录得久，产物一样）。
* **分辨率可独立调**（`--width/--height`）：视角由相机决定、与像素数无关，所以“同样的画面、
  更多像素”是原生支持的；各档分辨率在本机的单帧耗时见仓库根 `docs/mujoco-notes.md` 第 7.3 节。

本文件的循环 **故意照抄 `scripts/simulate.py` 的形状**（`torque` 变量、`if model.nu > 0`、
每步 `mj_step`），只多了 `rec.capture(data)` 一行。删掉的只有三处，都是“开窗口看”才需要的：
`launch_passive`、`viewer.sync()`、以及把仿真拖成实时的 `step_start` / `time.sleep`
（离线录制没必要等墙钟，跑满 CPU 更快；想保持实时就把那两行加回来，注释里有）。

用法：
    # 直接当“录一段”用：默认平地上零力矩跑 4 s，结果落在本目录的 output/ 下
    pixi run python @20260923_mujoco/scripts/visualization/examples/example_attach.py
    pixi run python @20260923_mujoco/scripts/visualization/examples/example_attach.py --seconds 5 --camera side
    pixi run python @20260923_mujoco/scripts/visualization/examples/example_attach.py --start rest --follow trunk
    # 复现“默认位形穿模被弹飞”（对照场景）
    pixi run python @20260923_mujoco/scripts/visualization/examples/example_attach.py \\
        --scene @20260923_mujoco/scenes/flat_scene_raw.xml --seconds 2
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import mujoco

# 本文件在 scripts/visualization/examples/ 下，先把 scripts/ 加进搜索路径才能 import visualization
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from visualization import CAMERAS, VideoRecorder  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent      # scripts/visualization/examples/

# 向上找**同时**含 scenes/ 与 models/ 的目录（不写死层数）
ROOT = next(
    p for p in pathlib.Path(__file__).resolve().parents
    if (p / "scenes").is_dir() and (p / "models").is_dir()
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=pathlib.Path, default=ROOT / "scenes/flat_scene.xml")
    parser.add_argument("--seconds", type=float, default=4.0, help="录多长（仿真时间）")
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--camera", choices=sorted(CAMERAS), default="iso")
    parser.add_argument("--follow", default=None, help="相机跟随的 body，如 trunk")
    parser.add_argument("--start", choices=("default", "rest"), default="default",
                        help="起飞状态：default=模型原样（和 simulate.py 一样，会自然趴下），rest=趴卧 keyframe")
    parser.add_argument("--out", type=pathlib.Path, default=HERE / "output/example_attach.mp4",
                        help="默认写到本目录的 output/ 下（库会自动建目录）")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    args = parser.parse_args()

    # 加载场景；默认的 flat_scene.xml 自带 <keyframe name="rest">，--start rest 依赖它
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    if args.start == "rest":
        if model.nkey == 0:                     # 场景里没有 keyframe 时早点报错，别默默录一段错的
            print(f"!! {args.scene.name} 里没有 keyframe，无法用 --start rest", file=sys.stderr)
            return 1
        mujoco.mj_resetDataKeyframe(model, data, 0)

    print(f"scene={args.scene.name}  start={args.start}  timestep={model.opt.timestep}  "
          f"目标 {args.seconds:g} s 仿真时间")

    # ② 建录制器（就在你的模型/数据建好之后，循环之前）；① 是文件开头的 import
    with VideoRecorder(model, args.out, fps=args.fps, width=args.width, height=args.height,
                       camera=args.camera, follow=args.follow) as rec:

        # ---- ↓↓↓ 以下就是"你自己的仿真程序"，跟录像库无关，想怎么写就怎么写 ↓↓↓ ----
        while data.time < args.seconds:
            torque = 0.0                    # 你的控制逻辑：任务只要求力矩全 0
            if model.nu > 0:
                data.ctrl[:] = torque

            mujoco.mj_step(model, data)
            rec.capture(data)               # ③ ←←← 相对你自己的循环，唯一新增的一行

            # 离线录制不必按墙钟等：跑满 CPU 更快。若需要保持实时（接硬件、或以后想开窗口看），
            # 就把 simulate.py 里那两行加回来：
            #     left = model.opt.timestep - (time.perf_counter() - step_start)
            #     if left > 0:
            #         time.sleep(left)
        # ---- ↑↑↑ 你的循环到此结束 ↑↑↑ ----

    print(f"仿真 {data.time:.2f} s，基座 z={data.qpos[2]:.4f} m，接触点 {data.ncon}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
