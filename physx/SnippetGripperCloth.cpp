//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates simple use of physx
//
// It creates a number of box stacks on a plane, and if rendering, allows the
// user to create new stacks and fire a ball from the camera position
// ****************************************************************************

#include <ctype.h>
#include <vector>

#include "PxPhysicsAPI.h"


#include "SnippetCommon/SnippetPrint.h"
#include "SnippetCommon/SnippetPVD.h"
#include "SnippetUtils/SnippetUtils.h"


using namespace physx;

bool					bRunSim = false;
PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;

PxMaterial*				gMaterial	= NULL;

PxPvd*                  gPvd        = NULL;

PxCloth*				gCloth      = NULL;
//maintain cloth state for rendering
std::vector< PxVec3 > 	gClothPos;
std::vector< PxVec3 > 	gClothNormal;
std::vector< PxU32 > 	gClothIndices;
PxU32* 					gIndices 	= NULL;

// two spheres for a tapered capsule and one by itself
PxClothCollisionSphere   gSpheres[3] = {
	PxClothCollisionSphere(PxVec3( 0.0f, 0.3f, 1.0f), 0.3f),
	PxClothCollisionSphere(PxVec3(-0.5f, 0.2f, 0.0f), 0.2f),
	PxClothCollisionSphere(PxVec3(-1.5f, 0.2f, 0.0f), 0.2f)
};

// a tetrahedron
PxClothCollisionTriangle gTriangles[4] = {
	PxClothCollisionTriangle(PxVec3(-0.3f, 0.0f, -1.3f), PxVec3( 0.3f, 0.6f, -1.3f), PxVec3( 0.3f, 0.0f, -0.7f)),
	PxClothCollisionTriangle(PxVec3( 0.3f, 0.6f, -1.3f), PxVec3(-0.3f, 0.6f, -0.7f), PxVec3( 0.3f, 0.0f, -0.7f)),
	PxClothCollisionTriangle(PxVec3(-0.3f, 0.6f, -0.7f), PxVec3(-0.3f, 0.0f, -1.3f), PxVec3( 0.3f, 0.0f, -0.7f)),
	PxClothCollisionTriangle(PxVec3(-0.3f, 0.0f, -1.3f), PxVec3(-0.3f, 0.6f, -0.7f), PxVec3( 0.3f, 0.6f, -1.3f)),
};

const PxU32 			nCollisionSpheres = 25;
const float				collisionSphereRadius = 0.02f;
//a grid of spheres for cloth collision
PxU32					gCollisionSphereIndices[nCollisionSpheres];
PxClothCollisionSphere  gCollisionSpheres[nCollisionSpheres]; 

PxTransform gClothPose = PxTransform(PxVec3(0), PxQuat(0*PxPi/4, PxVec3(0, 1, 0))) * PxTransform(PxVec3(0, 0.6f, 0));

PxRigidDynamic* createStick(const PxTransform& t)
{
	PxRigidDynamic* static_stick = PxCreateKinematic(*gPhysics, t, PxCapsuleGeometry(0.02f, 1.0f), *gMaterial, 7.8f);
	static_stick->setName("stick");
	gScene->addActor(*static_stick);
	return static_stick;
}

void createCloth()
{
	// create regular mesh
	PxU32 resolution = 16;	//segmentfault for a resolution more than 16... well, thats beacuse it supports at most 4096 points https://github.com/xarray/osgRecipes/blob/master/integrations/osgphysx3/SampleCloth.cpp
	PxU32 numParticles = resolution*resolution;
	PxU32 numQuads = (resolution-1)*(resolution-1);

	// create particles
	PxClothParticle* particles = new PxClothParticle[numParticles];
	PxVec3 center(0.5f, 0.0f, -0.3f);
	
	PxVec3 delta = 1.0f/(resolution-1) * PxVec3(1.0f, 0.0f, 0.6f);
	PxClothParticle* pIt = particles;

	PxU32 k=0;	//for collision spheres
	for(PxU32 i=0; i<resolution; ++i)
	{
		for(PxU32 j=0; j<resolution; ++j, ++pIt)
		{
			//this is the line fixing the top line by setting a zero inverse weight
			// pIt->invWeight = j+1<resolution ? 1.0f : 0.0f;
			pIt->invWeight = 1.0;
			pIt->pos = delta.multiply(PxVec3(PxReal(i), 
				0, -PxReal(j))) - center;

			if((i%4==0 || i==resolution-1) && (j%4==0 || j==resolution-1))
			{
				// printf("i: %u, j: %u\n", i, j);
				gCollisionSpheres[k] = PxClothCollisionSphere(pIt->pos, collisionSphereRadius);
				gCollisionSphereIndices[k] = i*resolution+j;
				k++;
			}
		}
	}

	// create quads
	PxU32* quads = new PxU32[4*numQuads];
	PxU32* iIt = quads;
	for(PxU32 i=0; i<resolution-1; ++i)
	{
		for(PxU32 j=0; j<resolution-1; ++j)
		{
			*iIt++ = (i+0)*resolution + (j+0);
			*iIt++ = (i+1)*resolution + (j+0);
			*iIt++ = (i+1)*resolution + (j+1);
			*iIt++ = (i+0)*resolution + (j+1);
		}
	}

	// create fabric from mesh
	PxClothMeshDesc meshDesc;
	meshDesc.points.count = numParticles;
	meshDesc.points.stride = sizeof(PxClothParticle);
	meshDesc.points.data = particles;
	meshDesc.invMasses.count = numParticles;
	meshDesc.invMasses.stride = sizeof(PxClothParticle);
	meshDesc.invMasses.data = &particles->invWeight;
	meshDesc.quads.count = numQuads;
	meshDesc.quads.stride = 4*sizeof(PxU32);
	meshDesc.quads.data = quads;

	// cook fabric
	PxClothFabric* fabric = PxClothFabricCreate(*gPhysics, meshDesc, PxVec3(0, -1, 0));

	delete[] quads;

	// create cloth and add to scene
	gCloth = gPhysics->createCloth(gClothPose, *fabric, particles, PxClothFlag::eSCENE_COLLISION);
	gCloth->setName("cloth");
	gScene->addActor(*gCloth);

	fabric->release();
	delete[] particles;

	// 240 iterations per/second (4 per-60hz frame)
	gCloth->setSolverFrequency(240.0f);

	// add virtual particles
	PxU32* indices = new PxU32[4*4*numQuads];
	PxU32* vIt = indices;
	for(PxU32 i=0; i<resolution-1; ++i)
	{
		for(PxU32 j=0; j<resolution-1; ++j)
		{
			*vIt++ = i*resolution + j;
			*vIt++ = i*resolution + (j+1);
			*vIt++ = (i+1)*resolution + (j+1);
			*vIt++ = 0;

			*vIt++ = i*resolution + (j+1);
			*vIt++ = (i+1)*resolution + (j+1);
			*vIt++ = (i+1)*resolution + j;
			*vIt++ = 0;

			*vIt++ = (i+1)*resolution + (j+1);
			*vIt++ = (i+1)*resolution + j;
			*vIt++ = i*resolution + j;
			*vIt++ = 0;

			*vIt++ = (i+1)*resolution + j;
			*vIt++ = i*resolution + j;
			*vIt++ = i*resolution + (j+1);
			*vIt++ = 0;
		}
	}

	// barycentric weights specifying virtual particle position
	PxVec3 weights = PxVec3(0.6, 0.2, 0.2);
	gCloth->setVirtualParticles(4*numQuads, indices, 1, &weights);

	delete[] indices;

	// add capsule (referencing spheres[0] and spheres[1] added later)
	// gCloth->addCollisionCapsule(1, 2);

	// add ground plane and tetrahedron convex (referencing planes added later)
	// gCloth->addCollisionConvex(0x01);
	// gCloth->addCollisionConvex(0x1e);

	// add collision spheres
	for(PxU32 i=0; i<nCollisionSpheres; ++i)
	{
		gCloth->addCollisionSphere(gCollisionSpheres[i]);
	}

	gCloth->setSelfCollisionDistance(0.01f);
	gCloth->setSelfCollisionStiffness(0.1f);

	gCloth->setClothFlag(PxClothFlag::eSCENE_COLLISION, true);

	gCloth->setInertiaScale( 0.5f );
	gCloth->setDampingCoefficient( PxVec3(0.2f, 0.2f, 0.2f) );
	gCloth->setDragCoefficient( 0.1f );
	// segmentfault when setting non-zero friction coefficient, why? seems requiring contact bodies
	gCloth->setFrictionCoefficient(0.5f);
}

PxRigidDynamic* 		gGripper 	= NULL;
PxRigidDynamic* 		gFinger1 	= NULL;
PxRigidDynamic* 		gFinger2 	= NULL;
PxD6Joint*				gJoint1 	= NULL;
PxD6Joint*				gJoint2 	= NULL;

void createGripper(const PxTransform& t)
{
	gGripper = PxCreateKinematic(*gPhysics, t, PxBoxGeometry(0.05f, 0.01f, 0.1f), *gMaterial, 2.7f);
	gGripper->setName("gripper");


	gFinger1 = PxCreateDynamic(*gPhysics, t.transform(PxTransform(0.0f, 0.11f, -0.08f)), PxBoxGeometry(0.05f, 0.1f, 0.02f), *gMaterial, 2.7f);
	gFinger1->setName("gripper_finger1");

	gFinger2 = PxCreateDynamic(*gPhysics, t.transform(PxTransform(0.0f, 0.11f, 0.08f)), PxBoxGeometry(0.05f, 0.1f, 0.02f), *gMaterial, 2.7f);
	gFinger2->setName("gripper_finger2");

	//slider joints, use D6 joint as it allows us to drive it
	PxD6JointDrive drive(100.0f, 2.0f, PX_MAX_F32, true);

	gJoint1 = PxD6JointCreate(*gPhysics, gGripper, PxTransform(PxVec3(0.0f, 0.01f, -0.08f)), 
			gFinger1, PxTransform(PxVec3(0.0f, -0.1f, 0.0f)));
	gJoint1->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
	gJoint1->setDrive(PxD6Drive::eZ, drive);

	gJoint2 = PxD6JointCreate(*gPhysics, gGripper, PxTransform(PxVec3(0.0f, 0.01f, 0.08f)), 
			gFinger2, PxTransform(PxVec3(0.0f, -0.1f, 0.0f)));
	gJoint2->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
	gJoint2->setDrive(PxD6Drive::eZ, drive);

	gScene->addActor(*gGripper);
	gScene->addActor(*gFinger1);
	gScene->addActor(*gFinger2);

	return;
}

void initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);
	groundPlane->setName("ground");

	
	createStick(PxTransform(0.0, 0.5f, 0.0));

	createCloth();

	createGripper(PxTransform(0.0, 0.05f, 0.0f));
}

void stepPhysics(bool interactive)
{
	PX_UNUSED(interactive);

	//update cloth global state
	if(gIndices == NULL)
	{
		//only get once...
		PxU32 nParticles = gCloth->getNbVirtualParticles();
		gIndices = new PxU32[4*nParticles];
		gCloth->getVirtualParticles(gIndices);
		gClothPos.resize(nParticles);
		gClothNormal.resize(nParticles);
		gClothIndices.assign(gIndices, gIndices+4*nParticles);
		// printf("length of gIndices: %d\n", gClothIndices.size());
	}

	//get particle data
	PxClothParticleData* pData = gCloth->lockParticleData();
	PxClothParticle* pParticles = pData->particles;
	//update the positions
   	for(PxU32 i=0;i < gClothPos.size();i++) 
	{
		//it seems that particle pos is only local to the cloth pose...
    	gClothPos[i] = gClothPose.transform(pParticles[i].pos);
	}
	pData->unlock();
	//update normals
	//update normals, this is triangles, how about QUAD?
	for(PxU32 i=0;i < gClothIndices.size();i+=3) {
		PxVec3 p1 = gClothPos[gClothIndices[i]];
		PxVec3 p2 = gClothPos[gClothIndices[i+1]];
		PxVec3 p3 = gClothPos[gClothIndices[i+2]];
		PxVec3 n  = (p2-p1).cross(p3-p1);

		gClothNormal[gClothIndices[i]]    += n/3.0f ; 
		gClothNormal[gClothIndices[i+1]]  += n/3.0f ; 
		gClothNormal[gClothIndices[i+2]]  += n/3.0f ;    
	}

	//update collision spheres positions
	for(PxU32 i=0;i < nCollisionSpheres; ++i)
	{
		gCollisionSpheres[i] = PxClothCollisionSphere(gClothPos[gCollisionSphereIndices[i]], collisionSphereRadius);
	}
	// seems the collision spheres might be unncessary after we have enough virtual particles for collision detection
	// the previous failure about collision is because of the transposed visualization, the post on 3.1 seems not completely correct!
	// gCloth->setCollisionSpheres(gCollisionSpheres, nCollisionSpheres);

	if(bRunSim)
	{
		gScene->simulate(1.0f/60.0f);
		gScene->fetchResults(true);
	}
	
}
	
void cleanupPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gCloth->release();
	gScene->release();
	gDispatcher->release();
	gPhysics->release();	
	PxPvdTransport* transport = gPvd->getTransport();
	gPvd->release();
	transport->release();
	
	gFoundation->release();

	if(gIndices != NULL)
	{
		delete[] gIndices;
	}
	
	printf("SnippetHelloWorld done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	// PhysX has occupied ASDW and disabled UP/DOWN/LEFT/RIGHT, use JKLI, O/P
	// printf("Pressed %u\n", static_cast<unsigned int>(key));
	switch(toupper(key))
	{
		case ' ':	bRunSim = !bRunSim;	break;
		case 'J':
			if(gGripper!=NULL)
			{
				gGripper->setKinematicTarget(gGripper->getGlobalPose().transform(PxTransform(-0.005f, 0, 0)));
			}	
			break;
		case 'L':
			if(gGripper!=NULL)
			{
				gGripper->setKinematicTarget(gGripper->getGlobalPose().transform(PxTransform(0.005f, 0, 0)));
			}	
			break;
		case 'I':	
			if(gGripper!=NULL)
			{
				gGripper->setKinematicTarget(gGripper->getGlobalPose().transform(PxTransform(0, 0.005f, 0)));
			}
			break;
		case 'K':	
			if(gGripper!=NULL)
			{
				gGripper->setKinematicTarget(gGripper->getGlobalPose().transform(PxTransform(0, -0.005f, 0)));
			}
			break;
		case 'O':
			if(gJoint1!=NULL && gJoint2!=NULL && gGripper!=NULL)
			{
				// printf("%f, %f, %f\n", gJoint1->getDrivePosition().p.x, gJoint1->getDrivePosition().p.y, gJoint1->getDrivePosition().p.z);
				gJoint1->setDrivePosition(gJoint1->getDrivePosition().transform(PxTransform(0, 0, -0.002f)));
				gJoint2->setDrivePosition(gJoint2->getDrivePosition().transform(PxTransform(0, 0, 0.002f)));

				//seems we need to set some pose to trigger the update, otherwise, setting a new drive position wont immediately apply the update
				gGripper->setKinematicTarget(gGripper->getGlobalPose());
			}	
			break;
		case 'P':
			if(gJoint1!=NULL && gJoint2!=NULL && gGripper!=NULL)
			{
				gJoint1->setDrivePosition(gJoint1->getDrivePosition().transform(PxTransform(0, 0, 0.002f)));
				gJoint2->setDrivePosition(gJoint2->getDrivePosition().transform(PxTransform(0, 0, -0.002f)));

				gGripper->setKinematicTarget(gGripper->getGlobalPose());
			}		
			break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
