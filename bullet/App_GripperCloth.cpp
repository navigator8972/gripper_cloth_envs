/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "App_GripperCloth.h"


const btScalar PI(3.1415926535897932384626433832795028841972);


void App_GripperCloth::initPhysics()
{
	m_guiHelper->setUpAxis(1);

    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

#ifdef USE_DEFORMABLE_BODY
    btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();

	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(deformableBodySolver);
	m_solver = sol;

	m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
	
	btVector3 gravity = btVector3(0, -9.81, 0);
	m_dynamicsWorld->setGravity(gravity);
	getDeformableDynamicsWorld()->getWorldInfo().m_gravity = gravity;
	
	//set to use implicit integration or not. 
	getDeformableDynamicsWorld()->setImplicit(false);
#else
	
	m_solver  = new btSequentialImpulseConstraintSolver();
	btSoftBodySolver* softBodySolver = 0;

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, softBodySolver);
	m_dynamicsWorld = world;
	// m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback, this, true);

	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	
	btVector3 gravity = btVector3(0, -9.81, 0);
	m_dynamicsWorld->setGravity(gravity);
	m_softBodyWorldInfo.m_gravity.setValue(0, -9.81, 0);

	m_softBodyWorldInfo.m_sparsesdf.Initialize();

	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	btVector3 worldAabbMin(-1000, -1000, -1000);
	btVector3 worldAabbMax(1000, 1000, 1000);

	m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, 32766);

	m_softBodyWorldInfo.m_broadphase = m_broadphase;
#endif

	
//    deformableBodySolver->setWorld(getDeformableDynamicsWorld());
    
    
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	{
		///create a few basic rigid bodies
		btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

		//groundShape->initializePolyhedralFeatures();
		//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

		m_collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -50, 0));
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	// create a stick
	{
		btCapsuleShape* stickShape = new btCapsuleShape(0.02, 1);
		// stickShape->setMargin(0.05);
		m_collisionShapes.push_back(stickShape);

		btTransform initTransform(btQuaternion(btVector3(0, 0, btScalar(1.0f)), btScalar(PI/2)), btVector3(0.0, 0.5f, 0.0));


		btScalar mass(0.);

		btRigidBody* stickBody = createRigidBody(mass, initTransform, stickShape, btVector4(0, 0, 0, 1));
		stickBody->setFriction(0.2f);
	}


	// create a cloth
	{
       const btScalar s = 0.2;
       const btScalar h = 0.6;

#ifdef USE_DEFORMABLE_BODY
		btSoftBody* psb = btSoftBodyHelpers::CreatePatch(getDeformableDynamicsWorld()->getWorldInfo(), btVector3(-s, h, -s),
											btVector3(+s, h, -s),
											btVector3(-s, h, +s),
											btVector3(+s, h, +s),
											20,20,
											// 2,2,
											0, true);
#else
		btSoftBody* psb = btSoftBodyHelpers::CreatePatch(m_softBodyWorldInfo, btVector3(-s, h, -s),
											btVector3(+s, h, -s),
											btVector3(-s, h, +s),
											btVector3(+s, h, +s),
											20,20,
											// 2,2,
											0, true);
#endif

		psb->getCollisionShape()->setMargin(0.022);

#ifndef USE_DEFORMABLE_BODY
		btSoftBody::Material* pm = psb->appendMaterial();
		pm->m_kLST = 0.4;
		pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
		psb->generateBendingConstraints(2, pm);
		psb->setTotalMass(0.1);

		psb->m_cfg.piterations = 30;
		psb->m_cfg.citerations = 30;
		psb->m_cfg.diterations = 30;

		//dynamic friction, alleviate drifting on the pole
		psb->m_cfg.kDF = 2;
		psb->generateClusters(32);
		psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RS;
		psb->m_cfg.collisions |= btSoftBody::fCollision::CL_SELF;
		// psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS
		// + btSoftBody::fCollision::CL_SELF
		// ;
#else
		psb->generateBendingConstraints(2);
		// psb->generateClusters(32);
		psb->setTotalMass(0.1);
		// psb->setSpringStiffness(100);
		// psb->setDampingCoefficient(10);
		psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
		psb->m_cfg.kCHR = 1; // collision hardness with rigid body
		psb->m_cfg.kDF = 1.5;  //dynamic friction, alleviate drifting on the pole
		psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
		// psb->m_cfg.collisions |= btSoftBody::fCollision::VF_DD;
#endif

		


#ifdef USE_DEFORMABLE_BODY		
		getDeformableDynamicsWorld()->addSoftBody(psb);
		btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(.1,.05, true);
		getDeformableDynamicsWorld()->addForce(psb, mass_spring);
		btDeformableNeoHookeanForce* neohookean = new btDeformableNeoHookeanForce(20,10);
		getDeformableDynamicsWorld()->addForce(psb, neohookean);
		getDeformableDynamicsWorld()->addForce(psb, new btDeformableGravityForce(gravity));
#else
		getSoftDynamicsWorld()->addSoftBody(psb);
#endif
   }


   //create a gripper
   	{
		btBoxShape* gripperBasisShape = createBoxShape(btVector3(0.05f, 0.01f, 0.1f));
		btTransform gripperBasisTransform;
		gripperBasisTransform.setIdentity();
		gripperBasisTransform.setOrigin(btVector3(0.0, 0.05f, 0.0f));
		m_collisionShapes.push_back(gripperBasisShape);

		btScalar mass(0.5);			//set a dynamic link and motorized it with another constraint
		m_gripperBase = createRigidBody(mass, gripperBasisTransform, gripperBasisShape, btVector4(0, 0, 1, 1));

		btBoxShape* gripperFinger1Shape = createBoxShape(btVector3(0.05f, 0.1f, 0.02f));
		m_collisionShapes.push_back(gripperFinger1Shape);
		btRigidBody* finger1 = createRigidBody(0.5, gripperBasisTransform*btTransform(btQuaternion::getIdentity(), btVector3(0.0f, 0.11f, -0.08f)), gripperFinger1Shape, btVector4(0, 0, 1, 1));

		btBoxShape* gripperFinger2Shape = createBoxShape(btVector3(0.05f, 0.1f, 0.02f));
		m_collisionShapes.push_back(gripperFinger2Shape);
		btRigidBody* finger2 = createRigidBody(0.5, gripperBasisTransform*btTransform(btQuaternion::getIdentity(), btVector3(0.0f, 0.11f, 0.08f)), gripperFinger2Shape, btVector4(0, 0, 1, 1));

		//constraint to the world, similar to MuJoCo
		m_gripperBaseJoint = new btGeneric6DofConstraint(*m_gripperBase, gripperBasisTransform, false);
		
		m_gripperBaseJoint->setLinearLowerLimit(btVector3(-10, -0.02, 0));
		m_gripperBaseJoint->setLinearUpperLimit(btVector3(10,  0.4, 0));
		m_gripperBaseJoint->setAngularLowerLimit(btVector3(0, 0, -0.00001));
		m_gripperBaseJoint->setAngularUpperLimit(btVector3(0, 0, 0.00001));
		m_gripperBaseJoint->setOverrideNumSolverIterations(30);

		m_gripperBaseJoint->getTranslationalLimitMotor()->m_enableMotor[0] = true;
		m_gripperBaseJoint->getTranslationalLimitMotor()->m_enableMotor[1] = true;
		m_gripperBaseJoint->getTranslationalLimitMotor()->m_enableMotor[3] = true;
		m_gripperBaseJoint->getTranslationalLimitMotor()->m_maxMotorForce[0] = 4000.0f;
		m_gripperBaseJoint->getTranslationalLimitMotor()->m_maxMotorForce[1] = 4000.0f;
		m_gripperBaseJoint->getTranslationalLimitMotor()->m_maxMotorForce[2] = 4000.0f;

		m_gripperBaseJoint->getRotationalLimitMotor(0)->m_enableMotor = true;
		m_gripperBaseJoint->getRotationalLimitMotor(1)->m_enableMotor = true;
		m_gripperBaseJoint->getRotationalLimitMotor(2)->m_enableMotor = true;
		m_gripperBaseJoint->getRotationalLimitMotor(0)->m_maxMotorForce = 4000.0f;
		m_gripperBaseJoint->getRotationalLimitMotor(1)->m_maxMotorForce = 4000.0f;
		m_gripperBaseJoint->getRotationalLimitMotor(2)->m_maxMotorForce = 4000.0f;

	   	//similar to PhysX
	   	m_gripperJoint1 = new btSliderConstraint(*m_gripperBase, *finger1, 
			btTransform(btQuaternion(btVector3(0, btScalar(1.0f), 0), btScalar(PI/2)), btVector3(0.0f, 0.01f, -0.08f)), 
		   	btTransform(btQuaternion(btVector3(0, btScalar(1.0f), 0), btScalar(PI/2)), btVector3(0.0f, -0.1f, 0.0f)), true);
		m_gripperJoint2 = new btSliderConstraint(*m_gripperBase, *finger2, 
			btTransform(btQuaternion(btVector3(0, btScalar(1.0f), 0), btScalar(PI/2)), btVector3(0.0f, 0.01f, 0.08f)), 
		   	btTransform(btQuaternion(btVector3(0, btScalar(1.0f), 0), btScalar(PI/2)), btVector3(0.0f, -0.1f, 0.0f)), true);

		//useful to make the constraints more stiff, because of PBD?
		m_gripperJoint1->setOverrideNumSolverIterations(30);
		m_gripperJoint2->setOverrideNumSolverIterations(30);

		m_gripperJoint1->setDbgDrawSize(btScalar(5.f));
		m_gripperJoint2->setDbgDrawSize(btScalar(5.f));

		m_gripperJoint1->setPoweredLinMotor(true);
		m_gripperJoint2->setPoweredLinMotor(true);

		//these limits are required to make setTargetLinMotorVelocity work
		m_gripperJoint1->setLowerLinLimit(-0.05f);
		m_gripperJoint1->setUpperLinLimit(0);
		m_gripperJoint1->setLowerAngLimit(0.0f);
		m_gripperJoint1->setUpperAngLimit(0.0f);
		m_gripperJoint1->setMaxLinMotorForce(4000.0f);


		m_gripperJoint2->setLowerLinLimit(0);
		m_gripperJoint2->setUpperLinLimit(0.05f);
		m_gripperJoint2->setLowerAngLimit(0.0f);
		m_gripperJoint2->setUpperAngLimit(0.0f);
		m_gripperJoint2->setMaxLinMotorForce(4000.0f);

		

		//true to disable link between constrained bodies
#ifdef USE_DEFORMABLE_BODY
		getDeformableDynamicsWorld()->addConstraint(m_gripperBaseJoint, true);
		getDeformableDynamicsWorld()->addConstraint(m_gripperJoint1, true);
		getDeformableDynamicsWorld()->addConstraint(m_gripperJoint2, true);
#else
		getSoftDynamicsWorld()->addConstraint(m_gripperBaseJoint, true);
		getSoftDynamicsWorld()->addConstraint(m_gripperJoint1, true);
		getSoftDynamicsWorld()->addConstraint(m_gripperJoint2, true);
#endif

		//initialize velocity
		m_gripperVelocity = btVector3(0, 0, 0);
		m_gripperFingerVelocity = 0.0f;
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

#ifndef USE_DEFORMABLE_BODY	
void App_GripperCloth::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;
	m_dynamicsWorld = 0;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
}
#else

#endif

void App_GripperCloth::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
		//set gripper velocity
		// btTransform curr_pose = m_gripperBase->getCenterOfMassTransform();
		// curr_pose.setOrigin(curr_pose.getOrigin()+m_gripperVelocity);
		// m_gripperBase->setCenterOfMassTransform(curr_pose);
		// printf("%f, %f, %f\n", m_gripperVelocity[0], m_gripperVelocity[1], m_gripperVelocity[2]);
		m_gripperBaseJoint->getTranslationalLimitMotor()->m_targetVelocity[0] = m_gripperVelocity[0];
		m_gripperBaseJoint->getTranslationalLimitMotor()->m_targetVelocity[1] = m_gripperVelocity[1];
		m_gripperBaseJoint->getTranslationalLimitMotor()->m_targetVelocity[2] = m_gripperVelocity[2];

		m_gripperJoint1->setTargetLinMotorVelocity(m_gripperFingerVelocity);
		m_gripperJoint2->setTargetLinMotorVelocity(-m_gripperFingerVelocity);

		m_dynamicsWorld->stepSimulation(deltaTime);

        // float internalTimeStep = 1. / 240.f;
        // m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
	}
}


CommonExampleInterface* App_GripperClothCreateFunc(CommonExampleOptions& options)
{
	return new App_GripperCloth(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(App_GripperClothCreateFunc)
