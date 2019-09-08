import mujoco_py as mjpy
import os

#mj_path, _ = mjpy.utils.discover_mujoco()
#xml_path = os.path.join(mj_path, 'model', 'empty_env.xml')
model = mjpy.load_model_from_path('gripper_cloth.xml')
sim = mjpy.MjSim(model)

viewer = mjpy.MjViewer(sim)

while True:
    sim.step()
    viewer.render()
