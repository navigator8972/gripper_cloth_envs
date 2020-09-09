import pybullet as p
import pybullet_data
from time import sleep
import sys
import os

import numpy as np

from pybullet_envs.robot_bases import MJCFBasedRobot

assets_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../assets')

class MyHumanoid(MJCFBasedRobot):

    def __init__(self, bullet_client, name='humanoid', basePosition=None, baseOrientation=None):
        MJCFBasedRobot.__init__(self, os.path.join(pybullet_data.getDataPath(), "mjcf",
                                                'humanoid_symmetric.xml'),name, action_dim=17, obs_dim=44)
        self._p = bullet_client
        self.humanoid_objs = bullet_client.loadMJCF(os.path.join(pybullet_data.getDataPath(), "mjcf",
                                                    'humanoid_symmetric.xml'),
                                    # flags=p.URDF_USE_SELF_COLLISION |
                                    # p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
                                    )
        self.parts, self.jdict, self.ordered_joints, self.robot_body = self.addToScene(bullet_client, self.humanoid_objs)
        for j in self.ordered_joints:
            self._p.setJointMotorControl2(self.humanoid_objs[j.bodyIndex], j.jointIndex, p.POSITION_CONTROL,
                                            targetVelocity=0,
                                            positionGain=1000,
                                            velocityGain=50,
                                            maxVelocity=1,
                                            force=100)
        
        if basePosition is not None and baseOrientation is not None:
            self.reset_pose(basePosition, baseOrientation)
        
        #set arm inital pose
        self.jdict['left_shoulder1'].set_state(np.pi/6, 0)
        self.jdict['left_shoulder2'].set_state(np.pi/6, 0)
        self.jdict['right_shoulder1'].set_state(-np.pi/6, 0)
        self.jdict['right_shoulder2'].set_state(-np.pi/6, 0)

        self.jdict['left_shoulder1'].set_position(np.pi/6)
        self.jdict['left_shoulder2'].set_position(np.pi/6)
        self.jdict['right_shoulder1'].set_position(-np.pi/6)
        self.jdict['right_shoulder2'].set_position(-np.pi/6)


        self.fixConstraintId = None
        return

    def fixBase(self):
        basePose = self.parts[self.robot_name].get_pose()
        # print(basePose)
        self.fixConstraintId = self._p.createConstraint(self.humanoid_objs[0], -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], basePose[:3])
        return
    
    def __del__(self):
        # MJCFBasedRobot.__del__()
        if self.fixConstraintId is not None:
            self._p.removeConstraint(self.fixConstraintId)
        return



def main():
    physicsClient = p.connect(p.GUI)
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

    p.setGravity(0, 0, -10)

    #data path to search object meshes
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    planeId = p.loadURDF("plane.urdf")

    clothId = p.loadSoftBody(fileName=os.path.join(assets_path, 'tshirt.obj'), basePosition=[0, 0, 1.35], baseOrientation=[ 0, 0, 0.7071068, 0.7071068 ], scale=.9, 
        collisionMargin=0.02, useMassSpring=1, mass=1, springElasticStiffness=10, springDampingStiffness=0.2, useBendingSprings=1, useFaceContact=1)
    # clothId = p.loadSoftBody(fileName=os.path.join(assets_path, 'tshirt_mia.obj'), basePosition=[0.02, 0, -0.05], baseOrientation=[ 0.5, 0.5, 0.5, 0.5 ], scale=.35, 
    #     collisionMargin=0.05, useMassSpring=1, mass=1, springElasticStiffness=8, springDampingStiffness=0.2, useBendingSprings=1)
    # cloth_mesh = p.getMeshData(clothId)
    # print(cloth_mesh[0], len(cloth_mesh[1]))
    # clothId = p.loadSoftBody(fileName=os.path.join(assets_path, 'hospitalgown_adaptivereduce.obj'), basePosition=[0, 0, 2.35], baseOrientation=[ 0, 0, 0.7071068, 0.7071068 ], scale=.9, 
    #     collisionMargin=0.02, useMassSpring=1, mass=1, springElasticStiffness=10, springDampingStiffness=0.2, useBendingSprings=1, useFaceContact=1)

    humanoid = MyHumanoid(p)
    humanoid.fixBase()

 
    runSimulation = False
    useRealTimeSimulation = 0

    cubeId = p.loadURDF("cube_small.urdf", [0.1, 0.05, 1.6], [1, 0, 0, 0], True, True)

    #use official anchor feature for deformable body
    # p.createSoftBodyAnchor(clothId ,3,cubeId,-1, [0.5,-0.5,0])


    # p.setRealTimeSimulation(1)

    while p.isConnected():
        
        keys = p.getKeyboardEvents()
        if ord('q') in keys and keys[ord('q')]&p.KEY_WAS_TRIGGERED:
            break
        if ord(' ') in keys and keys[ord(' ')]&p.KEY_WAS_TRIGGERED:
            runSimulation = not runSimulation
        if runSimulation:
            p.stepSimulation()
        # p.setGravity(0,0,-10)
        # sleep(1./240.)

    return

if __name__ == "__main__":
    main()     