import time

import mujoco
import mujoco.viewer
import pathlib as pl
cwd = pl.Path(__file__).parent

model = mujoco.MjModel.from_xml_path(str(cwd / "scene.xml"))
data = mujoco.MjData(model)

# info
print("nq =", model.nq)
print("nv =", model.nv)
print("nu =", model.nu)
print("# Model repr:", model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        mujoco.mj_step(model, data)
        viewer.sync()

        # info
        print("qpos =", data.qpos)
        print("qvel =", data.qvel)
        print("ctrl =", data.ctrl)
        print("time =", data.time)
        print("# Data repr:", data)

        time_left = model.opt.timestep - (time.time() - step_start)
        if time_left > 0:
            time.sleep(time_left)
