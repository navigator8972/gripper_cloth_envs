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

#include "btBulletDynamicsCommon.h"

//apparently, bullet is transiting its soft simulation from SoftBody to DeformableBody
//see the two words below. There is a small paragraph of comments in btDeformableMultiBodyDynamicsWorld stating the algorithm routine
//looks like the new one is more about conjugate-projection to deal with contact and use explicit internal forces, supporting corotational and neo-hookean.
//will PBD be eventually deprecated in Bullet?. I found the current DeformableBody is not stable, at least for clothing behaviors
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"

//from DeformableDemo
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"

//from DeformableDemo
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"

//from SoftDemo
#include "BulletSoftBody/btSoftSoftCollisionAlgorithm.h"

//from DeformableDemo
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "CommonInterfaces/CommonRigidBodyBase.h"

#define USE_DEFORMABLE_BODY

const btScalar PI(3.1415926535897932384626433832795028841972);

class btSoftRididCollisionAlgorithm;

struct App_GripperCloth : public CommonRigidBodyBase
{
	App_GripperCloth(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~App_GripperCloth() {}
	virtual void initPhysics();

	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	//these are from soft demo, look different from DeformableDemo, what are the differences?
	btAlignedObjectArray<btSoftSoftCollisionAlgorithm*> m_SoftSoftCollisionAlgorithms;

	btAlignedObjectArray<btSoftRididCollisionAlgorithm*> m_SoftRigidCollisionAlgorithms;

	btSoftBodyWorldInfo m_softBodyWorldInfo;

	btConstraintSolver* m_solver;

	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	//it is a bit messy to keep both soft and deformable worlds, try to condition the compilation later
#ifdef USE_DEFORMABLE_BODY
	virtual const btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld() const
    {
        return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
    }
    
    virtual btDeformableMultiBodyDynamicsWorld* getDeformableDynamicsWorld()
    {
        return (btDeformableMultiBodyDynamicsWorld*)m_dynamicsWorld;
    }

	virtual void renderScene()
    {
        CommonRigidBodyBase::renderScene();
        btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        
        for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        {
            btSoftBody* psb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
            {
                btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
                btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
            }
        }
    }
	
#else
	virtual const btSoftRigidDynamicsWorld* getSoftDynamicsWorld() const
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	}

	virtual btSoftRigidDynamicsWorld* getSoftDynamicsWorld()
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	}

	virtual void renderScene()
	{
		CommonRigidBodyBase::renderScene();
		btSoftRigidDynamicsWorld* softWorld = getSoftDynamicsWorld();

		for (int i = 0; i < softWorld->getSoftBodyArray().size(); i++)
		{
			btSoftBody* psb = (btSoftBody*)softWorld->getSoftBodyArray()[i];
			//if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
			{
				btSoftBodyHelpers::DrawFrame(psb, softWorld->getDebugDrawer());
				btSoftBodyHelpers::Draw(psb, softWorld->getDebugDrawer(), softWorld->getDrawFlags());
			}
		}
	}
#endif


};

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
#else
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;
	btSoftBodySolver* softBodySolver = 0;

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, softBodySolver);
	m_dynamicsWorld = world;
	// m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback, this, true);

	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	
	btVector3 gravity = btVector3(0, -9.81, 0);
	m_dynamicsWorld->setGravity(gravity);
	m_softBodyWorldInfo.m_gravity.setValue(0, -9.81, 0);

	m_softBodyWorldInfo.m_sparsesdf.Initialize();

	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;
#endif

	
//    deformableBodySolver->setWorld(getDeformableDynamicsWorld());
    
    
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));

	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	// create a stick
	{
		btCapsuleShape* stickShape = new btCapsuleShape(0.02, 0.5);
		m_collisionShapes.push_back(stickShape);

		btTransform initTransform(btQuaternion(btVector3(btScalar(1.0f), 0, 0), btScalar(PI/2)), btVector3(0.0, 0.5f, 0.0));

		btScalar mass(0.);

		createRigidBody(mass, initTransform, stickShape);
	}


	// create a cloth
	{
       const btScalar s = 0.25;
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
		btSoftBody* psb = btSoftBodyHelpers::CreatePatch(getSoftDynamicsWorld()->getWorldInfo(), btVector3(-s, h, -s),
											btVector3(+s, h, -s),
											btVector3(-s, h, +s),
											btVector3(+s, h, +s),
											20,20,
											// 2,2,
											0, true);
#endif

		psb->getCollisionShape()->setMargin(0.01);

#ifndef USE_DEFORMABLE_BODY
		btSoftBody::Material* pm = psb->appendMaterial();
		pm->m_kLST = 0.4;
		pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
		psb->generateBendingConstraints(2, pm);
#else
		psb->generateBendingConstraints(2);
#endif

		
		psb->setTotalMass(0.1);
		psb->setSpringStiffness(10);
		psb->setDampingCoefficient(0.3);
		psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
		psb->m_cfg.kCHR = 1; // collision hardness with rigid body
		psb->m_cfg.kDF = 1;
		psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;

#ifdef USE_DEFORMABLE_BODY		
		getDeformableDynamicsWorld()->addSoftBody(psb);
		// getDeformableDynamicsWorld()->addForce(psb, new btDeformableMassSpringForce());
		getDeformableDynamicsWorld()->addForce(psb, new btDeformableGravityForce(gravity));
#else
		getSoftDynamicsWorld()->addSoftBody(psb);
#endif
   }

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


CommonExampleInterface* App_GripperClothCreateFunc(CommonExampleOptions& options)
{
	return new App_GripperCloth(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(App_GripperClothCreateFunc)
