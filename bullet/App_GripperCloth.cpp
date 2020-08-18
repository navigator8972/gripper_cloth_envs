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


const btScalar PI(SIMD_PI);


void App_GripperCloth::initPhysics()
{
	m_guiHelper->setUpAxis(1);

    m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);


#ifdef USE_DEFORMABLE_BODY
	m_broadphase = new btDbvtBroadphase();

    btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();

	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
    sol->setDeformableSolver(deformableBodySolver);
	m_solver = sol;

	m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
	
	btVector3 gravity = btVector3(0, -9.81, 0);
	m_dynamicsWorld->setGravity(gravity);
	getDynamicsWorld()->getWorldInfo().m_gravity = gravity;
	
	//set to use implicit integration or not. 
	getDynamicsWorld()->setImplicit(false);
#else

	btVector3 worldAabbMin(-1000, -1000, -1000);
	btVector3 worldAabbMax(1000, 1000, 1000);

	m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, 32766);

	m_softBodyWorldInfo.m_broadphase = m_broadphase;
	
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

#endif
    
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	{
		///create a few basic rigid bodies
		btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

		//groundShape->initializePolyhedralFeatures();
		//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

		// m_collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -50, 0));
		btScalar mass(0.);

		bool isDynamic = (mass != 0.f);

        // btVector3 localInertia(0, 0, 0);
        // if (isDynamic)
        //     groundShape->calculateLocalInertia(mass, localInertia);
		
		// btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        // btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        // btRigidBody* body = new btRigidBody(rbInfo);
        // body->setFriction(0.5);

        // //add the ground to the dynamics world
        // m_dynamicsWorld->addRigidBody(body,1,1+2);

		btRigidBody* ground = createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
		ground->setFriction(0.5);
	}

	// create a stick
	{
		btCapsuleShape* stickShape = new btCapsuleShape(0.02, 1);
		// stickShape->setMargin(0.05);
		// m_collisionShapes.push_back(stickShape);

		btTransform stickTransform(btQuaternion(btVector3(0, 0, btScalar(1.0f)), btScalar(PI/2)), btVector3(0.0, 0.5f, 0.0));


		btScalar mass(0.);

		// bool isDynamic = (mass != 0.f);

        // btVector3 localInertia(0, 0, 0);
        // if (isDynamic)
        //     stickShape->calculateLocalInertia(mass, localInertia);
		
		// btDefaultMotionState* myMotionState = new btDefaultMotionState(stickTransform);
        // btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, stickShape, localInertia);
        // btRigidBody* body = new btRigidBody(rbInfo);
        // body->setFriction(0.3);

        // //add the ground to the dynamics world
        // m_dynamicsWorld->addRigidBody(body,1,1+2);

		btRigidBody* stick = createRigidBody(mass, stickTransform, stickShape, btVector4(0, 0, 1, 1));
		stick->setFriction(0.3);
	}


	// create a cloth
	{
       const btScalar s = 0.2;
       const btScalar h = 0.6;

#ifdef USE_DEFORMABLE_BODY
		btSoftBody* psb = btSoftBodyHelpers::CreatePatch(getDynamicsWorld()->getWorldInfo(), btVector3(-s, h, -s),
											btVector3(+s, h, -s),
											btVector3(-s, h, +s),
											btVector3(+s, h, +s),
											10,10,
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

		psb->getCollisionShape()->setMargin(0.01);
		
#ifdef USE_DEFORMABLE_BODY		

		psb->generateBendingConstraints(2);
		// psb->generateClusters(32);
		psb->setTotalMass(0.1);
		// psb->setSpringStiffness(100);
		// psb->setDampingCoefficient(10);
		psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
		psb->m_cfg.kCHR = 1; // collision hardness with rigid body
		psb->m_cfg.kDF = 2.;  //dynamic friction, alleviate drifting on the pole
		psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
		psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
    	psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
    	psb->m_cfg.collisions |= btSoftBody::fCollision::VF_DD;
		// psb->setSelfCollision(true);

		//disable deactivation
		// psb->setActivationState(DISABLE_DEACTIVATION);

		getDynamicsWorld()->addSoftBody(psb);
		btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(.15,.01, true);
		psb->setSpringStiffness(2);
		getDynamicsWorld()->addForce(psb, mass_spring);
		// btDeformableNeoHookeanForce* neohookean = new btDeformableNeoHookeanForce(20,10);
		// getDynamicsWorld()->addForce(psb, neohookean);
		getDynamicsWorld()->addForce(psb, new btDeformableGravityForce(gravity));
#else
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
		// psb->generateClusters(32);
		psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RS;
		psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDN;
    	psb->m_cfg.collisions |= btSoftBody::fCollision::SDF_RDF;
		// ;

		getDynamicsWorld()->addSoftBody(psb);
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
		btRigidBody* m_gripperBase = createRigidBody(mass, gripperBasisTransform, gripperBasisShape, btVector4(0, 0, 1, 1));
		

		btBoxShape* gripperFinger1Shape = createBoxShape(btVector3(0.05f, 0.1f, 0.02f));
		m_collisionShapes.push_back(gripperFinger1Shape);
		btRigidBody* finger1 = createRigidBody(0.5, gripperBasisTransform*btTransform(btQuaternion::getIdentity(), btVector3(0.0f, 0.11f, -0.08f)), gripperFinger1Shape, btVector4(0, 0, 1, 1));
		

		btBoxShape* gripperFinger2Shape = createBoxShape(btVector3(0.05f, 0.1f, 0.02f));
		m_collisionShapes.push_back(gripperFinger2Shape);
		btRigidBody* finger2 = createRigidBody(0.5, gripperBasisTransform*btTransform(btQuaternion::getIdentity(), btVector3(0.0f, 0.11f, 0.08f)), gripperFinger2Shape, btVector4(0, 0, 1, 1));
		
		m_gripperBase->setActivationState(DISABLE_DEACTIVATION);
		finger1->setActivationState(DISABLE_DEACTIVATION);
		finger2->setActivationState(DISABLE_DEACTIVATION);

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
		m_gripperJoint1->setOverrideNumSolverIterations(40);
		m_gripperJoint2->setOverrideNumSolverIterations(40);

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
		getDynamicsWorld()->addConstraint(m_gripperBaseJoint, true);
		getDynamicsWorld()->addConstraint(m_gripperJoint1, true);
		getDynamicsWorld()->addConstraint(m_gripperJoint2, true);

		// for multibody gripper, stick to rigid bodies now for the compatibility to PBD
		// btTransform gripperBasisTransform;
		// gripperBasisTransform.setIdentity();
		// gripperBasisTransform.setOrigin(btVector3(0.0, 0.05f, 0.0f));
		// m_gripper = createGripper(gripperBasisTransform);
		
		//initialize velocity
		m_gripperVelocity = btVector3(0, 0, 0);
		m_gripperFingerVelocity = 0.0f;
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

#ifdef USE_DEFORMABLE_BODY

btMultiBody* App_GripperCloth::createGripper(const btTransform& pose)
{
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	bool canSleep = false;
	bool fixedBase = false;
	float baseMass = 0.5;

	//for base
	btCollisionShape* gripperBasisShape = createBoxShape(btVector3(0.05f, 0.01f, 0.1f));
	gripperBasisShape->setMargin(0.001f);
	// m_collisionShapes.push_back(gripperBasisShape);
	gripperBasisShape->calculateLocalInertia(baseMass, baseInertiaDiag);

	btMultiBody* pMultiBody = new btMultiBody(2, baseMass, baseInertiaDiag, fixedBase, canSleep);
	pMultiBody->setBaseWorldTransform(pose);	

	btMultiBodyLinkCollider* gripperBasisCollider = new btMultiBodyLinkCollider(pMultiBody, -1);
	gripperBasisCollider->setCollisionShape(gripperBasisShape);
	gripperBasisCollider->setWorldTransform(pose);
	gripperBasisCollider->setFriction(0.2f);

	pMultiBody->setBaseCollider(gripperBasisCollider);
	m_dynamicsWorld->addCollisionObject(gripperBasisCollider,1,1+2);

	//for fingers
	btVector3 sliderJointAxis(0, 0, 1);
	float gripperFingerMass = 0.5;
	btVector3 gripperFingerInertiaDiag(0.f, 0.f, 0.f);
	btCollisionShape* gripperFingerShape = createBoxShape(btVector3(0.05f, 0.1f, 0.02f));
	gripperFingerShape->setMargin(0.001);
	gripperBasisShape->calculateLocalInertia(gripperFingerMass, gripperFingerInertiaDiag);
	pMultiBody->setupPrismatic(0, gripperFingerMass, gripperFingerInertiaDiag, -1, btQuaternion(0.f, 0.f, 0.f, 1.f), 
		sliderJointAxis, 
		btVector3(0.0, 0.01f, 0.1f-0.02f),		//parent com to current pivot offset
		btVector3(0.0, 0.1f, 0.0),					//current pivot to current com offset
		true											//disable parent collision
		);
	pMultiBody->setupPrismatic(1, gripperFingerMass, gripperFingerInertiaDiag, -1, btQuaternion(0.f, 0.f, 0.f, 1.f), 
		sliderJointAxis, 
		btVector3(0.0, 0.01f, -(0.1f-0.02f)),		//parent com to current pivot offset
		btVector3(0.0, 0.1f, 0.0),					//current pivot to current com offset
		true											//disable parent collision
		);
	
	//need to figure out local pose for collision bodies
	btAlignedObjectArray<btQuaternion> world_to_local;
    world_to_local.resize(pMultiBody->getNumLinks() + 1);
	btAlignedObjectArray<btVector3> local_origin;
    local_origin.resize(pMultiBody->getNumLinks() + 1);
    world_to_local[0] = pMultiBody->getWorldToBaseRot();
    local_origin[0] = pMultiBody->getBasePos();

	// printf("num of links %d\n", pMultiBody->getNumLinks());

	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
    {
        const int parent = pMultiBody->getParent(i);
        world_to_local[i + 1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent + 1];
        local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), pMultiBody->getRVector(i)));
    }

	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
    {
        btVector3 posr = local_origin[i + 1];
        
        btScalar quat[4] = {-world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w()};
        
		// m_collisionShapes.push_back(gripperFingerShape);
        
        btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);
        
        col->setCollisionShape(gripperFingerShape);
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(posr);
        tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
        col->setWorldTransform(tr);
        col->setFriction(0.2f);

		m_dynamicsWorld->addCollisionObject(col,1,1+2);
        
        pMultiBody->getLink(i).m_collider = col;
    }

	//prepare motors for two finger links


	pMultiBody->finalizeMultiDof();
    ///
    m_dynamicsWorld->addMultiBody(pMultiBody);

	return pMultiBody;
}

#endif

void App_GripperCloth::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization
    // removePickingConstraint();
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
#ifdef USE_DEFORMABLE_BODY
    // delete forces
    for (int j = 0; j < m_forces.size(); j++)
    {
        btDeformableLagrangianForce* force = m_forces[j];
        delete force;
    }
    m_forces.clear();
#endif
	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;

	delete m_solver;

	delete m_broadphase;

	delete m_dispatcher;

	delete m_collisionConfiguration;
}


void App_GripperCloth::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
		//set gripper velocity
		m_gripperBaseJoint->getTranslationalLimitMotor()->m_targetVelocity[0] = m_gripperVelocity[0];
		m_gripperBaseJoint->getTranslationalLimitMotor()->m_targetVelocity[1] = m_gripperVelocity[1];
		m_gripperBaseJoint->getTranslationalLimitMotor()->m_targetVelocity[2] = m_gripperVelocity[2];

		m_gripperJoint1->setTargetLinMotorVelocity(m_gripperFingerVelocity);
		m_gripperJoint2->setTargetLinMotorVelocity(-m_gripperFingerVelocity);

#ifndef USE_DEFORMABLE_BODY
		m_dynamicsWorld->stepSimulation(deltaTime);
#else
		m_dynamicsWorld->stepSimulation(deltaTime);
        // float internalTimeStep = 1. / 240.f;
        // m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
#endif
	}
}


CommonExampleInterface* App_GripperClothCreateFunc(CommonExampleOptions& options)
{
	return new App_GripperCloth(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(App_GripperClothCreateFunc)
