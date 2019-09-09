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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  
#define RENDER_SNIPPET
#ifdef RENDER_SNIPPET

#include <vector>

#include "PxPhysicsAPI.h"

#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"

using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);


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

void renderCallback()
{
	stepPhysics(true);

	Snippets::startRender(sCamera->getEye(), sCamera->getDir());

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);

	// printf("%d\n", nbActors);
	
	if(nbActors)
	{
		std::vector<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);

		//process each type of objects
		std::vector<PxRigidActor*> stickActors(0);
		std::vector<PxRigidActor*> clothActors(0);
		std::vector<PxRigidActor*> gripperActors(0);
		std::vector<PxRigidActor*> otherActors(0);

		for(PxU32 i = 0; i < nbActors; ++i)
		{
			if(strcmp(actors[i]->getName(), "stick") == 0)
			{
				stickActors.push_back(actors[i]);
			}
			else if(strcmp(actors[i]->getName(), "cloth") == 0)
			{
				clothActors.push_back(actors[i]);
			}
			else if(strcmp(actors[i]->getName(), "gripper") == 0)
			{
				gripperActors.push_back(actors[i]);
			}
			else
			{
				otherActors.push_back(actors[i]);
			}
		}

		if(stickActors.size()>0)
		{
			Snippets::renderActors(&stickActors[0], static_cast<PxU32>(stickActors.size()), true, PxVec3(0.75f, 0.75f, 0.75f));
		}
		if(clothActors.size()>0)
		{
			Snippets::renderActors(&clothActors[0], static_cast<PxU32>(clothActors.size()), true, PxVec3(0.75f, 0.0f, 0.0f));
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
