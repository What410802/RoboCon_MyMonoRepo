#!/usr/bin/env python3
"""把**任意** MuJoCo 仿真循环录成 MP4 的工具库（可插进已有脚本）。

MuJoCo 本身没有录像功能：官方 `simulate` 只能存单帧截图，`mujoco.Renderer` 只返回 RGB
帧数组。本模块把「离屏渲染 + 送 ffmpeg 编码」封装成可以直接插进你现有仿真脚本的类：

    import mujoco
    from visualization import VideoRecorder        # scripts/ 下的脚本这样导入（见文末）

    model = mujoco.MjModel.from_xml_path("scene.xml")
    data = mujoco.MjData(model)
    with VideoRecorder(model, "out.mp4", fps=50, camera="iso", follow="trunk") as rec:
        while running:
            ...你的控制与仿真...          # 想怎么算控制量都行
            mujoco.mj_step(model, data)
            rec.capture(data)             # 每步调一次即可，内部按 data.time 节流

要点
----
* `capture(data)` 用**仿真时间** `data.time` 判断该不该出新帧，所以调用方每步调、每 N 步调
  都一样，输出视频的时间轴始终与仿真时间一致（前提：每个仿真步至少调用一次）。
* **离屏尺寸不受模型 XML 限制**：MuJoCo 默认的离屏 framebuffer 只有 640x480
  （由 `<visual><global offwidth=".." offheight=".."/>` 控制），比它大的画面会让
  `Renderer` 直接报 `ValueError: Image width ... > framebuffer width ...`。
  本库会自动把这两个值调大到你要的 `width/height`，所以任意场景都能按要求尺寸录。
* 用 `with` 或显式 `close()` 收尾；`close()` 会等 ffmpeg 退出并报告结果。
* `follow="trunk"` 让相机始终看向该 body（狗跑起来时很有用）；不传则用预设机位。
* 渲染后端由 `MUJOCO_GL` 决定，且**必须在 `import mujoco` 之前设置**——MuJoCo 在那一刻
  就选定了后端，之后再改 `os.environ` 无效。本模块会在 `import mujoco` 前 `setdefault("MUJOCO_GL", "egl")`，
  但这只有在**本模块比 mujoco 先被导入**时才起作用。为了不依赖 import 顺序，仓库根的
  `pixi.toml` 已经把 `MUJOCO_GL = "egl"` 放在 `[activation.env]` 里（每个 pixi 环境都带上）。
  各后端的差异与实测数据见仓库根 `docs/mujoco-notes.md` 第 7 节（决策索引在 §7.6）。
* 依赖系统 `ffmpeg`（本机 /usr/bin/ffmpeg，带 libx264）。没有就在构造时直接报错。

命令行用法与接入示例都见 `examples/example_attach.py`。
其他脚本建议统一用包入口导入：

    from visualization import VideoRecorder          # scripts/ 下的脚本直接可用

（`visualization/__init__.py` 把 `VideoRecorder` / `CAMERAS` / `build_camera` 提到了包一级。）
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")   # 必须在 import mujoco 之前

import mujoco  # noqa: E402

# 预设机位：名字 -> (azimuth, elevation, distance, lookat)
CAMERAS: dict[str, tuple[float, float, float, tuple[float, float, float]]] = {
    "iso": (135.0, -20.0, 2.0, (0.0, 0.0, 0.15)),
    "front": (180.0, -8.0, 1.6, (0.0, 0.0, 0.12)),
    "side": (90.0, -8.0, 1.6, (0.0, 0.0, 0.12)),
    "top": (135.0, -75.0, 1.8, (0.0, 0.0, 0.1)),
}


def build_camera(
    model: mujoco.MjModel,
    preset: str = "iso",
    *,
    follow: str | None = None,
    azimuth: float | None = None,
    elevation: float | None = None,
    distance: float | None = None,
    lookat: tuple[float, float, float] | None = None,
) -> tuple[mujoco.MjvCamera, int | None]:
    """构造一个自由相机；返回 (camera, follow_body_id 或 None)。"""
    if preset not in CAMERAS:
        raise ValueError(f"未知机位 {preset!r}，可选：{sorted(CAMERAS)}")
    azimuth0, elevation0, distance0, lookat0 = CAMERAS[preset]

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.azimuth = azimuth0 if azimuth is None else azimuth
    camera.elevation = elevation0 if elevation is None else elevation
    camera.distance = distance0 if distance is None else distance
    camera.lookat[:] = lookat0 if lookat is None else lookat

    if follow is None:
        return camera, None
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, follow)
    if body_id < 0:
        raise ValueError(f"找不到名为 {follow!r} 的 body")
    return camera, body_id


class VideoRecorder:
    """把仿真过程写进 MP4。用法见模块 docstring。"""

    def __init__(
        self,
        model: mujoco.MjModel,
        path: str | Path,
        *,
        fps: float = 50.0,
        width: int = 960,
        height: int = 540,
        camera: str = "iso",
        follow: str | None = None,
        crf: int = 20,
        preset: str = "veryfast",
        ffmpeg: str = "ffmpeg",
        quiet: bool = False,
    ) -> None:
        if shutil.which(ffmpeg) is None:
            raise RuntimeError(
                "找不到 ffmpeg；MuJoCo 自身不带编码器，请先安装 ffmpeg（或 `pixi add ffmpeg`）"
            )

        self.model = model
        self.path = Path(path)
        self.fps = float(fps)
        self.width, self.height = int(width), int(height)
        self.frames = 0
        self._quiet = quiet

        self.camera, self.follow_id = build_camera(model, camera, follow=follow)

        self.path.parent.mkdir(parents=True, exist_ok=True)

        # 离屏 framebuffer 至少要装得下请求的尺寸：MuJoCo 默认只有 640x480
        # （由模型 XML 的 <visual><global offwidth/offheight> 控制），比它大的画面
        # 会让 Renderer 直接报 ValueError，这里不够就调大。
        global_vis = model.vis.global_
        global_vis.offwidth = max(global_vis.offwidth, self.width)
        global_vis.offheight = max(global_vis.offheight, self.height)

        self._proc: subprocess.Popen[bytes] | None = subprocess.Popen(
            [
                ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
                "-f", "rawvideo", "-pixel_format", "rgb24",
                "-video_size", f"{self.width}x{self.height}", "-framerate", f"{self.fps:g}",
                "-i", "-",                      # 从标准输入读原始帧
                "-an", "-c:v", "libx264", "-preset", preset,
                "-pix_fmt", "yuv420p", "-crf", str(crf),
                str(self.path),
            ],
            stdin=subprocess.PIPE,
        )
        self._renderer = mujoco.Renderer(model, self.height, self.width)
        self._next_t: float | None = None

    # -- 录制 ---------------------------------------------------------------
    def capture(self, data: mujoco.MjData, *, force: bool = False) -> bool:
        """按需渲染并写入一帧；返回本帧是否真的被写下。"""
        if self._proc is None:
            raise RuntimeError("recorder 已关闭")

        now = float(data.time)
        if self._next_t is None:
            self._next_t = now
        elif not force and now + 1e-9 < self._next_t:
            return False

        if self.follow_id is not None:
            self.camera.lookat[:] = data.xpos[self.follow_id]

        self._renderer.update_scene(data, camera=self.camera)
        assert self._proc.stdin is not None
        self._proc.stdin.write(self._renderer.render().tobytes())
        self.frames += 1

        self._next_t += 1.0 / self.fps
        if self._next_t <= now:                 # 调用方步长很大或隔很久才调一次：不追补旧帧
            self._next_t = now + 1.0 / self.fps
        return True

    @property
    def duration(self) -> float:
        """已录下的仿真时长（秒）。"""
        return self.frames / self.fps

    # -- 收尾 ---------------------------------------------------------------
    def close(self) -> None:
        if self._proc is None:
            return
        assert self._proc.stdin is not None
        self._proc.stdin.close()
        code = self._proc.wait()
        self._renderer.close()
        self._proc = None

        if not self._quiet:
            if self.frames == 0:
                print(f"!! 没有写任何帧，{self.path} 可能是空文件", flush=True)
            else:
                size = self.path.stat().st_size
                print(
                    f"已写入 {self.path}（{self.frames} 帧 / {self.duration:.1f} s / "
                    f"{self.width}x{self.height}@ {self.fps:g}fps / {size / 1024:.0f} KiB）",
                    flush=True,
                )
        if code != 0:
            raise RuntimeError(f"ffmpeg 退出码 {code}，编码失败")

    def __enter__(self) -> VideoRecorder:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()
