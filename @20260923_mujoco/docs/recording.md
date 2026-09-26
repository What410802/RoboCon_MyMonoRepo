# 录像工具：怎么接进自己的仿真脚本

> 背景知识在仓库文档里，这里只讲怎么用本任务的库：**MuJoCo 没有原生录像功能**（Python 侧 `mujoco.Renderer` 只返回 RGB 帧数组）见 [`../../docs/learn/mujoco.md`](../../docs/learn/mujoco.md) §6.4；离屏渲染的尺寸限制见同文件 §6.6；各后端的开销实测见 §7。

最早的 `record_video.py` 自带一个仿真循环（想录自己的控制逻辑就只能把逻辑抄进去）。现在**录**与**算**是分开的：真正干活的是库 `../scripts/visualization/mujoco_video.py` —— 它不关心你的循环长什么样，只要模型/数据是你自己的，并每个仿真步调一次 `capture(data)`（`visualization/__init__.py` 把它提成了包入口，所以 `scripts/` 下的脚本直接 `from visualization import VideoRecorder` 就行）：

```python
from visualization import VideoRecorder                   # ← ① 新增
...
with VideoRecorder(model, "out.mp4", fps=50, camera="iso") as rec:   # ← ② 新增
    while 你的条件:
        你的控制逻辑                                      # 一行都不用改
        mujoco.mj_step(model, data)
        rec.capture(data)                                # ← ③ 新增
```

现成可照抄的模板：`../scripts/simulate_record.py`（就是 `simulate.py` 接上录像的版本，无窗口）。

* **不需要窗口**：离屏渲染走 EGL（仓库根 `pixi.toml` 的 `[activation.env]` 已设 `MUJOCO_GL=egl`）。这个变量必须**在 `import mujoco` 之前**生效，脚本里再 `setdefault` 可能已经太晚；不设时的默认值（`glfw`）依赖显示服务，而且在双显卡机器上还可能落到另一块 GPU 上。
* 帧率、相机、编码、文件收尾全在库里；`capture()` 按**仿真时间** `data.time` 决定该不该出帧，所以输出 MP4 的时间轴 = 仿真时间，改 `fps=` 不用动循环，**与渲染耗时/机器快慢无关**（慢机器只是录得久，产物一模一样）。C++ 侧的 `../cpp_task2/src/record.h` 用的是同一条规则。
* **分辨率是独立可调的**：`--width/--height`，或库里 `VideoRecorder(width=…, height=…)`；视角由相机决定、与像素数无关。
* 完整的“接入”示例：`../scripts/visualization/examples/example_attach.py`（照抄 `simulate.py` 的循环形状，只多一行）。
* 想顺便在屏幕上开窗口看，才需要 `../scripts/visualization/examples/example_with_viewer.py`：`viewer.sync()` 每次都要等一个显示刷新周期，所以必须**降频**（每 10~25 步一次），否则仿真速度会被显示刷新钉住。
