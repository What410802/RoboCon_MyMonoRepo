import time
from pathlib import Path
cwd = Path(__file__).parent
project_path = cwd.parent

import mujoco
# import mujoco.viewer         # ← 只录像、不在屏幕上开窗口，就不需要它
from visualization import VideoRecorder      # 录像工具在 scripts/visualization/，直接跑脚本时可导

scene_path = project_path/"scenes/flat_scene.xml"
model = mujoco.MjModel.from_xml_path(str(scene_path))
data = mujoco.MjData(model)

out_path = project_path/"output/simulate_record.mp4"

# 相比 simulate.py，只多这一层 with（以及循环里的 rec.capture）
with VideoRecorder(model, out_path, fps=50, camera="iso") as rec:
    # 没有窗口就设不了 viewer.is_running()，改成按仿真时间收尾
    while data.time < 4.0:
        # 读取当前状态
        qpos = data.qpos.copy()
        qvel = data.qvel.copy()

        # 计算控制量
        torque = 0.0

        # 写入执行器控制输入
        if model.nu > 0:
            data.ctrl[:] = torque

        # 推进仿真
        mujoco.mj_step(model, data)

        # 更新 Viewer（只录像时没有窗口）
        # viewer.sync()
        rec.capture(data)        # ←←← 唯一需要新增的一行：该不该出帧由库按 data.time 决定
