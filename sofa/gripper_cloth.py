from stlib.scene import MainHeader, ContactHeader
from stlib.physics.rigid import Floor, Cube, RigidObject
from stlib.physics.constraints import FixedBox

from splib.numerics import Quat, Vec3, vadd
#from gripper import Gripper

def ListToString(in_list):
    return ' '.join([str(n) for n in in_list])

def Gripper(parentNode=None, **kwargs):
    selfNode = parentNode.createChild("Gripper")
    translation = kwargs.pop('translation')
    rotation = kwargs.pop('rotation')
    color = kwargs.pop('color')
    #WARNING: Quat createFromEuler will modify the input Euler list!!!
    rotation_copy = [r for r in rotation]
    rotBase=Quat.createFromEuler(rotation_copy, inDegree=True)

    transBase = Vec3(translation)
    scale = 0.05

    dimBase = Vec3([1.*scale, .2*scale, 2*scale])
    dimLink = Vec3([1.*scale, 2.*scale, .4*scale])
    offsetLink1 = Vec3([0.0, 2.2*scale, -1.6*scale])
    offsetLink2 = Vec3([0.0, 2.2*scale, 1.6*scale])
   
    translation_base = transBase.rotateFromQuat(rotBase).toList()
    translation_link1 = vadd(transBase.toList(), offsetLink1.rotateFromQuat(rotBase).toList())
    translation_link2 = vadd(transBase.toList(), offsetLink2.rotateFromQuat(rotBase).toList())

    base = Cube(selfNode, name='base', uniformScale=dimBase.toList(), translation=translation_base, rotation=rotation, color=color)
    #WARNING: rotateFromQuat is also an in-place operation, why do they do this?!
    attachOffset_translation1 = [0.0, .2*scale, -1.6*scale]
    attachOffset_translation2 = [0.0, .2*scale, 1.6*scale]

    baseAttaches = base.createChild('baseAttaches')
    baseAttaches.createObject('MechanicalObject', name='attaches', template='Rigid3d', 
      # translation2=attachOffset_translation1+attachOffset_translation2, rotation2=rotation+rotation
      position=ListToString(attachOffset_translation1)+ ' 0 0 0 1 ' + ListToString(attachOffset_translation2) + ' 0 0 0 1'
      )
    baseAttaches.createObject('RigidRigidMapping')
    base.getChild('collision').getObject("TTriangleModel").findData('group').value = '1 2'
    base.getChild('collision').getObject("TLineModel").findData('group').value = '1 2'
    base.getChild('collision').getObject("TPointModel").findData('group').value = '1 2'

    baseInput = base.createChild('baseInput')
    baseInput.createObject('MechanicalObject', name='input', template='Rigid3d', 
      translation=translation_base, rotation=rotation
      )
#     baseInput.createObject('JointSpringForceField', object1='@./', object2='@../', 
#       spring='BEGIN_SPRING 0 0 FREE_AXIS 0 0 0 0 0 0  KS_T 20000 30000  KS_R 150000 200000  KD 1  END_SPRING',
#       name='base_input', template='Rigid3d')
    baseInput.createObject('AttachConstraint', indices1='0', name='attachConstraint', indices2='0', 
      velocityFactor='1.0', responseFactor='1.0', object1='@./', object2='@../', positionFactor='1.0', constraintFactor='1', twoWay='false')

    link1 = Cube(selfNode, name='link1', uniformScale=dimLink.toList(), translation=translation_link1, rotation=rotation, color=color)
    attachOffset_translation = [0.0, -2.*scale, 0]
    link1Attaches = link1.createChild('link1Attaches')
    link1Attaches.createObject('MechanicalObject', name='attaches', template='Rigid3d', 
      position=ListToString(attachOffset_translation)+ ' 0 0 0 1 '
      )
    link1Attaches.createObject('RigidRigidMapping')
    link1.getChild('collision').getObject("TTriangleModel").findData('group').value = '1'
    link1.getChild('collision').getObject("TLineModel").findData('group').value = '1'
    link1.getChild('collision').getObject("TPointModel").findData('group').value = '1'


    link2 = Cube(selfNode, name='link2', uniformScale=dimLink.toList(), translation=translation_link2, rotation=rotation, color=color)
    attachOffset_translation = [0.0, -2.*scale, 0]
    link2Attaches = link2.createChild('link2Attaches')
    link2Attaches.createObject('MechanicalObject', name='attaches', template='Rigid3d', 
      position=ListToString(attachOffset_translation)+ ' 0 0 0 1 '
      )
    link2Attaches.createObject('RigidRigidMapping')
    link2.getChild('collision').getObject("TTriangleModel").findData('group').value = '2'
    link2.getChild('collision').getObject("TLineModel").findData('group').value = '2'
    link2.getChild('collision').getObject("TPointModel").findData('group').value = '2'

    link1.createObject('JointSpringForceField', object1="@../base/baseAttaches", object2="@./link1Attaches", spring='BEGIN_SPRING 0 0 FREE_AXIS 1 1 1 0 0 0  KS_T 20000 30000  KS_R 150000 200000  KD 100  REST_T 50 50 0.0 END_SPRING', name='joint_link1', template='Rigid3d')
    link2.createObject('JointSpringForceField', object1="@../base/baseAttaches", object2="@./link2Attaches", spring='BEGIN_SPRING 1 0 FREE_AXIS 1 1 1 0 0 0  KS_T 20000 30000  KS_R 150000 200000  KD 100  REST_T 0.0 50 50 END_SPRING', name='joint_link2', template='Rigid3d')
    # link1.createObject('AttachConstraint', indices1='0', name='attachConstraint', indices2='0', 
    #     velocityFactor='1.0', responseFactor='1.0', object1='@./link1Attaches', object2='@../base/baseAttaches', positionFactor='1.0', constraintFactor='1', twoWay='true')
    # link2.createObject('AttachConstraint', indices1='0', name='attachConstraint', indices2='1', 
    #     velocityFactor='1.0', responseFactor='1.0', object1='@./link2Attaches', object2='@../base/baseAttaches', positionFactor='1.0', constraintFactor='1', twoWay='true')

    GripperController(selfNode, base, link1, link2)
    return selfNode

def Stick(parentNode=None, **kwargs):
      # <Node name="Stick">
      #       <MeshObjLoader name="loader" filename="mesh/cylinder.obj" />
      #       <MeshTopology src="@loader" />
      #       <MechanicalObject src="@loader" name="stick_mech" scale3d='0.02 0.2 0.02' translation="1 0 1" rotation="0 90 0" />
      #       <Triangle simulated="false" moving="false" contactFriction="100"/>
      #       <LineCollisionModel simulated="false" moving="false" contactFriction="100"/>
      #       <PointCollisionModel simulated="false"  moving="false" contactFriction="100"/>
      #       <OglModel src="@loader" name="VisualModel" color="white" scale3d='@stick_mech.scale3d' translation="1 0 1" rotation="0 90 0"/>
      # </Node>
      selfNode = parentNode.createChild("Stick")
      '''
      selfNode.createObject('MeshObjLoader', name='loader', filename="mesh/cylinder.obj")
      selfNode.createObject('MeshTopology', src='@loader')
      selfNode.createObject('MechanicalObject', src='@loader', scale3d='.02 0.2 .02', name='stick_mech', translation=[1, 0, 0], rotation=[0, 0, 90])

      selfNode.createObject('Triangle', moving=False, simulated=False, contactFriction=100)
      selfNode.createObject('LineCollisionModel', moving=False, simulated=False, contactFriction=100)
      selfNode.createObject('PointCollisionModel', moving=False, simulated=False, contactFriction=100)
      selfNode.createObject('OglModel', src='@loader', name='VisualModel', color='white', scale3d='@stick_mech.scale3d', translation=[1, 0, 0], rotation=[0, 0, 90])
      '''
      selfNode.createObject('MeshObjLoader', name='loader', filename='mesh/cylinder.obj')
      selfNode.createObject('MeshTopology', src='@loader')
      #2.5 -0.05 1.6 vs 1 0 0 
      selfNode.createObject('MechanicalObject', scale3d='0.02 0.2 0.02', src='@loader', translation='1 0 0', rotation='0 0 90', name='stick_mech')
      selfNode.createObject('Triangle', moving='false', contactFriction='100', simulated='false')
      selfNode.createObject('LineCollisionModel', moving='false', contactFriction='100', simulated='false')
      selfNode.createObject('PointCollisionModel', moving='false', contactFriction='100', simulated='false')
      selfNode.createObject('OglModel', scale3d='@stick_mech.scale3d', src='@loader', name='VisualModel', color='white', translation='@stick_mech.translation', rotation='@stick_mech.rotation')

      return selfNode

def Cloth(parentNode=None, **kwargs):
      # <Node name="Cloth Dynamic Mesh (red)">
      #   <EulerImplicitSolver name="cg_odesolver" printLog="0" />
      #   <CGLinearSolver name="linear solver" template="GraphScattered" iterations="20" tolerance="1e-09" threshold="1e-9" />
      #   <MeshGmshLoader name="loader" filename="mesh/square3.msh" createSubelements="true" />
      #   <MechanicalObject src="@loader" name="DOF" template="Vec3d" translation="1.5 0 1.1" rotation="90 0 0" scale="1" />
      #   <MinProximityIntersection alarmDistance="0.005" contactDistance="0.001"/>
      #   <FixedConstraint name="Fixed dof" template="Vec3d" indices="0 1" />
      #   <UniformMass name="Mass" template="Vec3d" vertexMass="0.01" />
      #   <TriangleSetTopologyContainer src="@loader" name="Topology Container" />
      #   <TriangleSetTopologyModifier name="Topology Modifier" />
      #   <TriangleSetTopologyAlgorithms template="Vec3d" name="Topology Algorithms" />
      #   <TriangleSetGeometryAlgorithms template="Vec3d" name="Geometry Algorithms" />
      #   <TriangularBendingSprings name="FEM-Bend" template="Vec3d" stiffness="300" damping="1" />
      #   <TriangularFEMForceField name="FEM" template="Vec3d" method="large" poissonRatio=".3" youngModulus="1000" />
      #   <TriangleModel name="models" />
      #   <!-- <Triangle name="ClothTriangle"  selfCollision="1" contactStiffness="5" contactFriction="10"/> -->
      #   <LineModel name="ClothLine"  selfCollision="1" contactStiffness="8" contactFriction="1" group="1"/>
      #   <!-- <PointModel name="ClothPoint" selfCollision="1" contactStiffness="30" contactFriction="100" group="1"/> -->

      #   <OglModel name="Visual" template="ExtVec3f" color="red" />
      #   <IdentityMapping name="Mapping" template="Vec3d,ExtVec3f" input="@." output="@Visual" />
      # </Node>

      selfNode = parentNode.createChild('Cloth Dynamic Mesh (red)')

      selfNode.createObject('EulerImplicitSolver', printLog='0', rayleighStiffness='0.1', name='cg_odesolver', rayleighMass='0.1')
      selfNode.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-09', name='linear solver', iterations='20', template='GraphScattered')
      selfNode.createObject('MeshGmshLoader', createSubelements='true', name='loader', filename='mesh/square3.msh')
      #1.5 0 1.1 vs 0 0.05 -0.5
      selfNode.createObject('MechanicalObject', src='@loader', scale='1', name='DOF', template='Vec3d', translation='-0.5 0.05 -0.5', rotation='90 0 0')
      selfNode.createObject('MinProximityIntersection', contactDistance='0.1', alarmDistance='0.2')
      selfNode.createObject('UniformMass', vertexMass='0.01', name='Mass', template='Vec3d')
      selfNode.createObject('TriangleSetTopologyContainer', src='@loader', name='Topology Container')
      selfNode.createObject('TriangleSetTopologyModifier', name='Topology Modifier')
      selfNode.createObject('TriangleSetTopologyAlgorithms', name='Topology Algorithms', template='Vec3d')
      selfNode.createObject('TriangleSetGeometryAlgorithms', name='Geometry Algorithms', template='Vec3d')
      selfNode.createObject('TriangularBendingSprings', damping='1', name='FEM-Bend', stiffness='10', template='Vec3d')
      selfNode.createObject('TriangularFEMForceField', method='large', template='Vec3d', name='FEM', poissonRatio='.3', youngModulus='800')
    #   selfNode.createObject('TriangleModel', selfCollision='0', group='10', name='ClothTriangle', bothSide='1', contactStiffness='3', contactFriction='1')
      selfNode.createObject('LineModel', contactFriction='1', selfCollision='1', group='10', contactStiffness='3', name='ClothLine')
      selfNode.createObject('PointModel', contactFriction='1', selfCollision='0', group='11', contactStiffness='5', name='ClothPoint')
      selfNode.createObject('OglModel', color='red', name='Visual', template='ExtVec3f')
      selfNode.createObject('IdentityMapping', input='@.', name='Mapping', template='Vec3d,ExtVec3f', output='@Visual')

      '''
      selfNode = parentNode.createChild('Cloth Dynamic Mesh (red)')
      selfNode.createObject('EulerImplicitSolver', name='cg_odesolver', printLog=False, rayleighStiffness=0.1, rayleighMass=0.1)
      selfNode.createObject('CGLinearSolver', name='linear solver', template='GraphScattered', iterations=20, tolerance=1e-9, threshold=1e-9)
      selfNode.createObject('MeshGmshLoader', name='loader', filename='mesh/square3.msh', createSubelements=True)
      # selfNode.createObject('MeshObjLoader', name='loader', filename='mesh/square3.obj')
      selfNode.createObject('MechanicalObject', src='@loader', name='DOF', template='Vec3d', translation=[-0.5, 0.05, -0.5], rotation=[90, 0, 0], scale3d=[1, 1, 1])
      # 
      ContactHeader(selfNode, alarmDistance=.05, contactDistance=.02, frictionCoef=0.1)
      selfNode.createObject('MinProximityIntersection', contactDistance='0.2', alarmDistance='0.1')

      selfNode.createObject('UniformMass', name='mass', template='Vec3d', vertexMass=0.005)
      selfNode.createObject('TriangleSetTopologyContainer', src='@loader', name='TopoContainer')
      selfNode.createObject('TriangleSetTopologyModifier', name='TopoModifier')
      # selfNode.createObject('TriangleSetTopologyAlgorithms', template='Vec3d', name='TopoAlgorithm')
      selfNode.createObject('TriangleSetGeometryAlgorithms', template='Vec3d', name='GeomAlgorithm', recomputeTrianglesOrientation=True)
      selfNode.createObject('TriangularBendingSprings', name='FEM-Bend', template='Vec3d', stiffness=10, damping=.1)
      selfNode.createObject('TriangularFEMForceField', name='FEM', template='Vec3d', method='large', poissonRatio=.1, youngModulus=800)

      selfNode.createObject('TriangleCollisionModel', name='ClothTriangle', selfCollision="1", contactStiffness=3, contactFriction=1, bothSide=True)
      # selfNode.createObject('LineCollisionModel', name='ClothLine', selfCollision="1", contactStiffness=5, contactFriction=1)
      selfNode.createObject('PointCollisionModel', name='ClothPoint', selfCollision="0", contactStiffness=3, contactFriction=1)
      
      # TriangularSurface = selfNode.createChild('TriangularSurface')
      # TriangularSurface.createObject('TriangleSetTopologyContainer', name='Container')
      # TriangularSurface.createObject('TriangleSetTopologyModifier', name='Modifier')
      # TriangularSurface.createObject('Tetra2TriangleTopologicalMapping', input='@../TopoContainer', output='@Container')
      # TriangularSurface.createObject('Triangle', selfCollision=True, contactStiffness=3, contactFriction=1, bothSide=True)
      # # TriangularSurface.createObject('Line', selfCollision=True, contactStiffness=3, contactFriction=1)
      # TriangularSurface.createObject('Point', selfCollision=True, contactStiffness=3, contactFriction=1)


      selfNode.createObject('OglModel', name='Visual', template='ExtVec3d', color='red')
      selfNode.createObject('IdentityMapping', name='Mapping', template='Vec3d,ExtVec3d', input='@.', output='@Visual')
      #we need this to make the cloth subject to gravity, cannot find it in the working converted scene but from cloth_place.py, what's magic behind this?
      #not necessarily, some setup works without it...
      # selfNode.createObject('UncoupledConstraintCorrection', compliance='100')
      '''
      return selfNode

def createScene(rootNode):
#     rootNode.findData('dt').value = '0.01'
#     rootNode.findData('gravity').value = '0 -9.81 0'

    rootNode.createObject('VisualStyle', displayFlags='showVisual')
    rootNode.createObject('RequiredPlugin', name='SofaOpenglVisual')
    rootNode.createObject('BruteForceDetection')
    rootNode.createObject('DefaultContactManager')
    rootNode.createObject('DefaultPipeline')
    rootNode.createObject('MinProximityIntersection', contactDistance='0.005', alarmDistance='0.01')
    rootNode.createObject('CollisionResponse', name='Response', response='FrictionContact')

    Gripper(rootNode,
          color=[0.0,1.0,0.0],
          translation=[0.0, -0.75, 0.0],
          rotation=[0, 0, 0],
          isAStaticObject=True)

#     Floor(rootNode, name="Floor",
#           color=[1.0,1.0,0.0],
#           translation=[0.0,-10.0,0.0],
#           isAStaticObject=True)

#     Cube(rootNode, name="Support",
#           uniformScale=20.0,
#           color=[1.0,1.0,0.0],
#           totalMass=0.03,
#           volume=20,
#           inertiaMatrix=[1000.0,0.0,0.0,0.0,1000.0,0.0,0.0,0.0,1000.0],
#           translation=[0.0,-80.0,10.0],
#           isAStaticObject=True)
      #prepare stick and cloth
    Cloth(rootNode)
    Stick(rootNode)

    return rootNode

# -*- coding: utf-8 -*-
import Sofa

def getTranslated(points, vec):
    r=[]
    for v in points:
        r.append( [v[0]+vec[0], v[1]+vec[1], v[2]+vec[2]] )
    return r

class GripperController(Sofa.PythonScriptController):
    def __init__(self, node, base, link1, link2):
        self.base = base
        self.link1 = link1
        self.link2 = link2
        self.name = "GripperController"

    def onKeyPressed(self,c):
        dir = [0.0,0.0,0.0]
        switch = 0

        # UP key :
        if ord(c)==19:
            dir = [0.0,0.01,0.0]
        # DOWN key : rear
        elif ord(c)==21:
            dir = [0.0,-0.01,0.0]
        # LEFT key : left
        elif ord(c)==18:
            dir = [0.01,0.0,0.0]
        elif ord(c)==20:
            dir = [-0.01,0.0,0.0]
        #+/-: open/close
        elif ord(c) == 45:
            switch = -1
        elif ord(c) == 43:
            switch = 1

        mecaobject = self.base.getChild('baseInput').getObject('input')    
        position = getTranslated( mecaobject.position,  dir )
        mecaobject.position = position
        
        if switch != 0:
            #modify RigidRigidMapping to simulate actuator. not physical but didnt find a way to change the rest position of JointSpringForceField
            attaches = self.base.getChild('baseAttaches').getObject('RigidRigidMapping')
            attach_pos1 = attaches.initialPoints[0]
            attach_pos2 = attaches.initialPoints[1]
            attach_pos1[2] = attach_pos1[2] - switch*0.01
            attach_pos2[2] = attach_pos2[2] + switch*0.01
            attaches.initialPoints = [attach_pos1, attach_pos2]