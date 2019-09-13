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
#define RENDER_SNIPPET
#ifdef RENDER_SNIPPET

#include <vector>
#include <string>

#include "PxPhysicsAPI.h"

#include "SnippetRender/SnippetRender.h"
#include "SnippetRender/SnippetCamera.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);

extern PxU32* 					gIndices;
extern std::vector< PxVec3 > 	gClothPos;
extern std::vector< PxVec3 > 	gClothNormal;
extern std::vector< PxU32 > 	gClothIndices;

const PxU32 					nCollisionSpheres = 25;
const float						collisionSphereRadius=0.02f;
extern PxClothCollisionSphere  	gCollisionSpheres[]; 

namespace
{
Snippets::Camera*	sCamera;

void motionCallback(int x, int y)
{
	sCamera->handleMotion(x, y);
}

void keyboardCallback(unsigned char key, int x, int y)
{
	if(key==27)
		exit(0);

	if(!sCamera->handleKey(key, x, y))
		keyPress(key, sCamera->getTransform());
}

void mouseCallback(int button, int state, int x, int y)
{
	sCamera->handleMouse(button, state, x, y);
}

void idleCallback()
{
	glutPostRedisplay();
}

void RenderCloth(const std::vector<PxVec3>& pos, const std::vector<PxVec3>& normal, const std::vector< PxU32 >& indices) 
{
	//not sure if normals are correct for GL_QUAD, use GL_POINT for now
	glEnableClientState(GL_VERTEX_ARRAY);
	// glEnableClientState(GL_NORMAL_ARRAY);

	glVertexPointer(3, GL_FLOAT, sizeof(PxVec3), &(pos[0].x));
	// glNormalPointer(GL_FLOAT, sizeof(PxVec3), &(normal[0].x));
	glPointSize(5.0f);
	glDrawElements(GL_POINTS, indices.size(), GL_UNSIGNED_INT, &indices[0]);

	// glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	return;
}

void RenderSpheres()
{
	for(PxU32 i=0; i<nCollisionSpheres; ++i)
	{
		//Sphere
		glPushMatrix();
		glTranslatef(gCollisionSpheres[i].pos.x, gCollisionSpheres[i].pos.y, gCollisionSpheres[i].pos.z);
		glScalef(collisionSphereRadius,collisionSphereRadius,collisionSphereRadius);
		glutSolidSphere(1, 10, 10);		
		glPopMatrix();		
	}
	return;
}

void RenderClothActors(PxActor** actors, const PxU32 numActors, bool shadows, const PxVec3 & color)
{
	for(PxU32 i = 0; i < numActors; ++i)
	{
		PxCloth* pCloth = static_cast<PxCloth*>(actors[i]);
		// PxU32 nParticles = pCloth->getNbVirtualParticles();
		// printf("Number of cloth particles: %d\n", nParticles);

		// for(size_t j=0; j<nParticles; ++j)
		// {
		// 	printf("%u ", pIndices[j]);
		// }
		// printf("\n");

		//only deal with cloth object
		if(strcmp(pCloth->getName(), "cloth")==0)
		{
			// printf("Length of cloth state variables: %u, %u, %u\n", gClothPos.size(), gClothNormal.size(), gClothIndices.size());
			RenderCloth(gClothPos, gClothNormal, gClothIndices);
			RenderSpheres();
		}
	}

	return;
}

void renderCallback()
{
	stepPhysics(true);

	Snippets::startRender(sCamera->getEye(), sCamera->getDir());

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	PxU32 nbClothActors = scene->getNbActors(PxActorTypeFlag::eCLOTH);
	if(nbActors && nbClothActors)
	{
		std::vector<PxRigidActor*> rigidActors(nbActors);
		std::vector<PxActor*> clothActors(nbClothActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&rigidActors[0]), nbActors);
		scene->getActors(PxActorTypeFlag::eCLOTH, &clothActors[0], nbActors);

		//process each type of objects
		std::vector<PxRigidActor*> stickActors(0);
		std::vector<PxRigidActor*> gripperActors(0);
		std::vector<PxRigidActor*> otherActors(0);

		// printf("Number of rigid/cloth actors: %d/%d.\n", nbActors, nbClothActors);

		for(PxU32 i = 0; i < nbActors; ++i)
		{
			if(strcmp(rigidActors[i]->getName(), "stick") == 0)
			{
				stickActors.push_back(rigidActors[i]);
			}
			else if(std::string(rigidActors[i]->getName()).rfind("gripper", 0) == 0)
			{
				gripperActors.push_back(rigidActors[i]);
			}
			else
			{
				otherActors.push_back(rigidActors[i]);
			}
		}

		if(stickActors.size()>0)
		{
			Snippets::renderActors(&stickActors[0], static_cast<PxU32>(stickActors.size()), true, PxVec3(0.75f, 0.75f, 0.75f));
		}
		if(clothActors.size()>0)
		{
			// printf("Will render %u cloth actors.\n", clothActors.size());
			// Snippets::renderActors(&clothActors[0], static_cast<PxU32>(clothActors.size()), true, PxVec3(0.75f, 0.0f, 0.0f));
			RenderClothActors(&clothActors[0], static_cast<PxU32>(clothActors.size()), true, PxVec3(0.75f, 0.0f, 0.0f));
		}
		if(gripperActors.size()>0)
		{
			Snippets::renderActors(&gripperActors[0], static_cast<PxU32>(gripperActors.size()), true, PxVec3(0.0f, 0.75f, 0.0f));
		}
		if(otherActors.size()>0)
		{
			Snippets::renderActors(&otherActors[0], static_cast<PxU32>(otherActors.size()), true, PxVec3(0.15f, 0.15f, 0.15f));
		}
		
		// Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, PxVec3(1.0f, 0.75f, 0.0f));
	}

	Snippets::finishRender();
}


void exitCallback(void)
{
	delete sCamera;
	cleanupPhysics(true);
}
}

void renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(1.0f, 1.0f, 2.0f), PxVec3(-0.6f,-0.7f,-1.0f));

	Snippets::setupDefaultWindow("PhysX Snippet HelloWorld");
	Snippets::setupDefaultRenderState();

	glutIdleFunc(idleCallback);
	glutDisplayFunc(renderCallback);
	glutKeyboardFunc(keyboardCallback);
	glutMouseFunc(mouseCallback);
	glutMotionFunc(motionCallback);
	motionCallback(0,0);

	atexit(exitCallback);

	initPhysics(true);
	glutMainLoop();
}
#endif
