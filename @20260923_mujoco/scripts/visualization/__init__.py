"""录像工具的对外入口 —— 让 `scripts/` 下的脚本可以直接：

    from visualization import VideoRecorder

真正干活的模块是 `mujoco_video`（离屏渲染 + 管道给 ffmpeg），这里只是把常用名字提到包一级，
不需要用户记模块名。`scripts/` 目录本身就在 Python 的搜索路径上（它是脚本所在目录），
所以**在 `scripts/` 下的脚本里这一行直接可用**；`examples/` 子目录里的脚本要先
`sys.path` 把 `scripts/` 加进来（见 `examples/example_attach.py` 开头）。

命令行用法见 `examples/example_attach.py`（它也是零力矩录像的命令行入口）。
分辨率/帧率的成本与“为什么帧率与机器快慢无关”见仓库根 `docs/learn/mujoco.md` 第 7 节。
"""

from .mujoco_video import CAMERAS, VideoRecorder, build_camera

__all__ = ["CAMERAS", "VideoRecorder", "build_camera"]
