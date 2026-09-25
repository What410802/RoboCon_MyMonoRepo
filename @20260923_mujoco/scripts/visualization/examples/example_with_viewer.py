#!/usr/bin/env python3
"""示例（可选补充）：**一边开 viewer 看、一边录像**。

注意：**录像本身不需要窗口**（默认走 EGL 离屏渲染）。正常情况下请看 `example_attach.py`——
那个才是“把录像接进你自己的循环”的最小示例。本文件只解决额外情况：我还想同时用
`launch_passive` 看画面，此时才会碰到下面这些坑。

## 结论：不难，加 3 行就行；但要让它跑在实时，得顺手改两处

各调用的开销量级（**具体数值是机器相关的**，本机实测见仓库根 `docs/mujoco-notes.md` 第 7 节，
用 `scripts/agent_scripts/render_cost.py` 可在本机复现）：

| 调用 | 量级 | 换算 |
| --- | --- | --- |
| `mujoco.mj_step()` | 微秒级 | 500 Hz 也几乎不占 CPU |
| 离屏 `render()` + 回读 | **毫秒级** | 录像链路里的大头 |
| 录像整链路 `capture()` | 比渲染多 ~1~2 ms（编码+管道） | 50 fps 通常可接受 |
| `viewer.sync()` | **十几毫秒**（等一个刷新周期） | 每步调它 → 被钉在刷新率上，只剩 ~10% 实时 |

跑完 3 s 仿真所需的墙钟（`sync` 每 25 步一次 + 截止时间节流）：

| 配置 | 相对实时 |
| --- | --- |
| 纯离屏 + 录像 50 fps（`--no-viewer`） | **>100%**（比实时还快） |
| 开窗口，不录像 | ~100% |
| 开窗口 + 录像 10 fps | ~95% |
| 开窗口 + 录像 50 fps | ~85% |
| 开窗口，**每步** `sync`（= 原 `simulate.py`） | **~10%** |

所以：

1. 加录像（①②③三行）**代码上没难度**，库负责节流、编码、关文件；
2. 但要“一边看一边录还实时”，**输入得降配**：录像帧率降到 10 fps，或者干脆不要窗口
   （`--no-viewer`，50 fps 也没问题）；
3. 真正的瓶颈不在录像，而在 `viewer.sync()` —— 你的循环按 500 Hz 在调它，被刷新节流卡在 10% 实时。
   改成**每 N 步刷一次**（默认 `--sync-every 25` ≈ 20 Hz）就恢复了；
4. 节流用**截止时间**（目标墙钟时刻 = 起点 + `data.time`，可自我纠偏），而不是“只补本步亏欠”，
   否则 `time.sleep(1.7 ms)` 实际会睡 4~5 ms，偏差会累积
   （两种写法的实测对比见 `docs/mujoco-notes.md` 第 7.2 节）。

相对你原来的 `simulate.py`，最少只多三处：

    from visualization import VideoRecorder                      # ① 导入
    with mujoco.viewer.launch_passive(model, data) as viewer:
        with VideoRecorder(model, out, fps=10) as rec:           # ② 建录制器
            while viewer.is_running():
                ...
                mujoco.mj_step(model, data)
                rec.capture(data)                                # ③ 每步调一次，内部按时间节流
                viewer.sync()

两个坑：

- **渲染后端由 `MUJOCO_GL` 决定，且必须在 `import mujoco` 之前生效**（后端在那一刻就选定）。
  Linux 下不设时默认是 `glfw`：它走**显示的 GL**，需要显示服务；在双显卡机器上它还可能落到
  另一块 GPU 上（差异可达数倍，见 `docs/mujoco-notes.md` 第 7 节），所以**不要靠默认值**。
  本仓库在根 `pixi.toml` 的 `[activation.env]` 里统一设了 `egl`。
- 开窗口的进程退出时偶发 `segmentation fault`，或 `GLFWError: EGL: Failed to clear current
  context ...` 之类的清理告警（GLFW 与已存在的 EGL 上下文在同进程收尾时的冲突）；
  MP4 在崩溃前已由 `close()` 写完，产物不受影响，纯离屏脚本不会出现。

用法：
    pixi run python @20260923_mujoco/scripts/visualization/examples/example_with_viewer.py --seconds 3 --fps 10
    pixi run python @20260923_mujoco/scripts/visualization/examples/example_with_viewer.py --no-viewer  # 50 fps 离屏录像
    pixi run python @20260923_mujoco/scripts/visualization/examples/example_with_viewer.py --no-record  # 只开窗口
"""

from __future__ import annotations

import argparse
import contextlib
import pathlib
import sys
import time

import mujoco
import mujoco.viewer

# 本文件在 scripts/visualization/examples/ 下，先把 scripts/ 加进搜索路径才能 import visualization
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from visualization import CAMERAS, VideoRecorder  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent      # scripts/visualization/examples/

# 脚本可能被放在 scripts/ 下任意层级，向上找**同时**含 scenes/ 与 models/ 的目录
ROOT = next(
    p for p in pathlib.Path(__file__).resolve().parents
    if (p / "scenes").is_dir() and (p / "models").is_dir()
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=pathlib.Path, default=ROOT / "scenes/flat_scene.xml")
    parser.add_argument("--seconds", type=float, default=3.0,
                        help="录多长（**仿真时间**，秒）；开窗口时 3 s 仿真可能要 7 s 墙钟")
    parser.add_argument("--fps", type=float, default=50.0,
                        help="录像帧率；开窗口时想保持实时就降到 10")
    parser.add_argument("--camera", choices=sorted(CAMERAS), default="iso")
    parser.add_argument("--follow", default=None, help="相机跟随的 body，如 trunk")
    parser.add_argument("--no-viewer", action="store_true", help="不开窗口，纯离屏")
    parser.add_argument("--no-record", action="store_true",
                        help="只开窗口不录像（用来量 viewer 自身的开销）")
    parser.add_argument("--sync-every", type=int, default=25,
                        help="每隔多少仿真步刷新一次 viewer（默认 25 ≈ 20 Hz；设 1 = 每步都刷，会被显示刷新钉住）")
    parser.add_argument("--width", type=int, default=960, help="录像分辨率（与视角无关）")
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--pacing", choices=("deadline", "naive"), default="deadline",
                        help="实时节流方式：deadline=按目标墙钟时刻自我纠偏（推荐）；naive=只补本步亏欠")
    parser.add_argument("--out", type=pathlib.Path, default=HERE / "output/record_with_viewer.mp4",
                        help="默认写到本目录的 output/ 下（库会自动建目录）")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)

    viewer_ctx = (
        contextlib.nullcontext(None) if args.no_viewer
        else mujoco.viewer.launch_passive(model, data)
    )
    rec_ctx = (
        contextlib.nullcontext(None) if args.no_record
        else VideoRecorder(model, args.out, fps=args.fps, width=args.width, height=args.height,
                           camera=args.camera, follow=args.follow)
    )

    with viewer_ctx as viewer, rec_ctx as recorder:
        started = time.perf_counter()
        step = 0
        while (viewer is None or viewer.is_running()) and data.time < args.seconds:
            step_start = time.perf_counter()

            data.ctrl[:] = 0.0                       # 任务要求：力矩全 0
            mujoco.mj_step(model, data)
            if recorder is not None:
                recorder.capture(data)               # 每步调一次，内部按仿真时间节流

            step += 1
            if viewer is not None and step % args.sync_every == 0:
                viewer.sync()

            left = (
                (started + data.time) - time.perf_counter()      # 目标 = 起点 + 已仿真时间
                if args.pacing == "deadline"
                else model.opt.timestep - (time.perf_counter() - step_start)
            )
            if left > 0:
                time.sleep(left)

    wall = time.perf_counter() - started
    print(f"仿真 {data.time:.2f} s / 墙钟 {wall:.2f} s = {data.time / wall:.0%} 实时"
          f"（接触点 {data.ncon}，基座 z={data.qpos[2]:.4f} m）")
    print(f"viewer：{'未开窗口' if args.no_viewer else '已开窗口并自动关闭'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
