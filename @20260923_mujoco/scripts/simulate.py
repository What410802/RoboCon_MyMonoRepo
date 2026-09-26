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
        # 但这一圈里 viewer.sync() 要等一个刷新周期（本机实测中位 23 ms）≫ timestep(0.002 s)，
        # 所以单线程每步都 sync 最多只有 ~0.09x 实时，下面那句 WARN 基本每圈都会触发：
        # 这正是上游把 SIMULATE_DT 放大到 0.005 的原因，也是任务 3 改双缓冲的动机。
        # 想看 1x：把 sync 降频（如每 10~25 步一次，见 scripts/visualization/examples/），
        # 或直接用 python/main.py、cpp_task2 的 --mode view。
        time_left = model.opt.timestep - (time.time() - step_start)
        if time_left > 0:
            time.sleep(time_left)
        # else:
        #     print("[WARN] Simulation speed decreased.")
