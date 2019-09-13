import mujoco_py as mjpy
import os


class MyMujocoViewer(mjpy.MjViewer):

    def __init__(self, sim):
        super().__init__(sim)
        self.sim = sim
        return
    
    def key_callback(self, window, key, scancode, action, mods):
        super().key_callback(window, key, scancode, action, mods)

        if key == ord('A') or key == ord('a'):
            self.sim.data.ctrl[0] += 0.002
        elif key == ord('D') or key == ord('d'):
            self.sim.data.ctrl[0] -= 0.002
        elif key == ord('X') or key == ord('x'):
            self.sim.data.ctrl[1] -= 0.002
        elif key == ord('W') or key == ord('w'):
            self.sim.data.ctrl[1] += 0.002
        elif key == ord('O') or key == ord('o'):
            self.sim.data.ctrl[2] += 0.002
            self.sim.data.ctrl[3] -= 0.002
        elif key == ord('P') or key == ord('p'):
            self.sim.data.ctrl[2] -= 0.002
            self.sim.data.ctrl[3] += 0.002
        return

#mj_path, _ = mjpy.utils.discover_mujoco()
#xml_path = os.path.join(mj_path, 'model', 'empty_env.xml')
model = mjpy.load_model_from_path('gripper_cloth.xml')
sim = mjpy.MjSim(model)

viewer = MyMujocoViewer(sim)

while True:
    sim.step()
    viewer.render()
