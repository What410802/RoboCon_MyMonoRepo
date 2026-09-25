import time
import pathlib as pl
cwd = pl.Path(__file__).parent
project_path = cwd.parent

import mujoco
import mujoco.viewer

scene_path = project_path/"scenes/flat_scene.xml"
model = mujoco.MjModel.from_xml_path(str(scene_path))
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

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

        # 更新 Viewer
        viewer.sync()

        # 让显示速度大致接近真实时间
        time_left = model.opt.timestep - (time.time() - step_start)
        if time_left > 0:
            time.sleep(time_left)
        # else:
        #     print("[WARN] Simulation speed decreased.")
