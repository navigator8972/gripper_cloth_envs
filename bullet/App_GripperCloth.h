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

#ifndef APP_GRIPPERCLOTH_H
#define APP_GRIPPERCLOTH_H

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
#include "CommonInterfaces/CommonDeformableBodyBase.h"

#define USE_DEFORMABLE_BODY

class btSoftRididCollisionAlgorithm;

#ifndef USE_DEFORMABLE_BODY	
struct App_GripperCloth : public CommonRigidBodyBase
#else
struct App_GripperCloth : public CommonDeformableBodyBase
#endif
{
	
#ifndef USE_DEFORMABLE_BODY
	App_GripperCloth(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
#else
	App_GripperCloth(struct GUIHelperInterface* helper)
		: CommonDeformableBodyBase(helper)
	{
	}
#endif
	virtual ~App_GripperCloth() {}
	virtual void initPhysics();
	virtual void exitPhysics();
	virtual void stepSimulation(float deltaTime);
	virtual btMultiBody* createGripper(const btTransform& pose);

	void resetCamera()
	{
		float dist = 1;
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

	btRigidBody*                    m_gripperBase;
    btGeneric6DofConstraint*        m_gripperBaseJoint;
	btSliderConstraint*             m_gripperJoint1;
	btSliderConstraint*             m_gripperJoint2;

    btVector3   m_gripperVelocity;
    btScalar    m_gripperFingerVelocity;

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
		CommonDeformableBodyBase::renderScene();
		// btDeformableMultiBodyDynamicsWorld* deformableWorld = getDeformableDynamicsWorld();
        
        // for (int i = 0; i < deformableWorld->getSoftBodyArray().size(); i++)
        // {
        //     btSoftBody* psb = (btSoftBody*)deformableWorld->getSoftBodyArray()[i];
        //     {
        //         btSoftBodyHelpers::DrawFrame(psb, deformableWorld->getDebugDrawer());
        //         btSoftBodyHelpers::Draw(psb, deformableWorld->getDebugDrawer(), deformableWorld->getDrawFlags());
        //     }
        // }
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

class CommonExampleInterface* App_GripperClothCreateFunc(struct CommonExampleOptions& options);

#endif  //APP_GRIPPERCLOTH_H
