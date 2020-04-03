# gripper_cloth_envs
SOFA/PhysX/MuJoCo/Bullet environments for testing gripper and cloth interaction



### SOFA: [Simulation Open Framework Architecture](https://github.com/sofa-framework/sofa)

To use:

1. Compile SOFA (tested on v19.06) and [SoftRobots](https://project.inria.fr/softrobot/) plugin;

2. Move obj files in assets to $SOFA_ROOT_DIR/src/share/mesh/ because the loader seems to always load from there...

3. Run SOFA simulator **runSofa** from **build** directory; the installed one cannot correctly load the python script;

4. Load the python script by opening it as a scene; Press Animate button to start the simulation;

5. Use CTRL+UP/DOWN/LEFT/RIGHT/+/- to move and open/close the gripper.



### [PhysX 3.4](https://github.com/NVIDIAGameWorks/PhysX-3.4) (Need to join NVIDIAGameWorks organization)

To use:

1. Build PhysX 3.4; Note that [PxCloth is deprecated after 3.4 and NvCloth is now the product for cloth simulation;](https://developer.nvidia.com/clothing)

2. Modify the path in env.sh to point to your PhysX SDK folder and source env.sh;

3. mkdir build and cd build and cmake ..

4. make and run ./SnippetGripperCloth

5. Press SPACE to run the simulation; use J/K/L/I/O/P to move and open/close the gripper.


### [MuJoCo 2.0.0](http://www.mujoco.org/index.html) and [mujoco_py](https://github.com/openai/mujoco-py)

To use:

1. Install mujoco_py;

2. Run the python script;

3. Use A/S/D/W/O/P to move and open/close the gripper.

Note: the simulation window got stuck if we choose to render cloth texture. Looks like an issue of mujoco_py because the original mujoco example works.


### [Bullet3](https://github.com/bulletphysics/bullet3) 

To use:

1. Build bullet3; Tested on [commits](https://github.com/bulletphysics/bullet3/commit/ec2b6dd920135a5df804d521727cc06446a6a3bd) on Apr 3rd, 2020;

2. Modify the path in env.sh to point to your Bullet SDK folder and source env.sh;

3. mkdir build and cd build and cmake ..

4. make and run ./App_GripperClothGui

5. Press SPACE to run the simulation; use UP/DOWN/LEFT/RIGHT/O/P to move and open/close the gripper.

Note: The tested commits introduce new ways of simulating soft dynamics as DeformableMultiBody,
in contrast to original SoftBody. The environment uses a macro USE_DEFORMABLE_BODY in App_GripperCloth.h to disable/enable the old solver.








