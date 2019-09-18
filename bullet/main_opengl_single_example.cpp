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

#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "Utils/b3Clock.h"

#include "OpenGLWindow/SimpleOpenGL3App.h"
#include <stdio.h>
#include "ExampleBrowser/OpenGLGuiHelper.h"

#include "LinearMath/btTransform.h"
#include "App_GripperCloth.h"

CommonExampleInterface* example;
SimpleOpenGL3App* app = NULL;

int gSharedMemoryKey = -1;
bool gRunSimulation = false;

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove(float x, float y)
{
	bool handled = false;
	handled = example->mouseMoveCallback(x, y);
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback(x, y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback = 0;
static void OnMouseDown(int button, int state, float x, float y)
{
	bool handled = false;

	handled = example->mouseButtonCallback(button, state, x, y);
	if (!handled)
	{
		if (prevMouseButtonCallback)
			prevMouseButtonCallback(button, state, x, y);
	}
}

static void OnKeyboardPressed(int key, int state)
{	
	if(app == NULL || example == NULL)
	{
		return;
	}

	//process events for buttons pressed...

	// printf("%d, %d\n", key, state);
	if(state)
	{
		//only handle events triggered by pushing down the button
		switch(key)
		{
			case B3G_ESCAPE: app->m_window->setRequestExit();break;

			case B3G_SPACE: gRunSimulation = !gRunSimulation;break;

			case B3G_LEFT_ARROW: ((App_GripperCloth*)example)->m_gripperVelocity = btVector3(-0.05f, 0, 0);	break;

			case B3G_RIGHT_ARROW: ((App_GripperCloth*)example)->m_gripperVelocity = btVector3(0.05f, 0, 0); break;
			
			case B3G_UP_ARROW: ((App_GripperCloth*)example)->m_gripperVelocity = btVector3(0, -0.05f, 0);	break;

			case B3G_DOWN_ARROW: ((App_GripperCloth*)example)->m_gripperVelocity = btVector3(0, 0.05f, 0);	break;

			case 'o':
			case 'O':
				((App_GripperCloth*)example)->m_gripperFingerVelocity = btScalar(0.05);
				break;

			case 'p':
			case 'P':
				((App_GripperCloth*)example)->m_gripperFingerVelocity = btScalar(-0.05);
				break;

		}
	}
	else
	{
		switch(key)
		{
			//remember to clear the velocity command
			case B3G_LEFT_ARROW: ((App_GripperCloth*)example)->m_gripperVelocity = btVector3(0, 0, 0);break;

			case B3G_RIGHT_ARROW: ((App_GripperCloth*)example)->m_gripperVelocity = btVector3(0, 0, 0); break;

			case B3G_UP_ARROW: ((App_GripperCloth*)example)->m_gripperVelocity = btVector3(0, 0, 0);	break;

			case B3G_DOWN_ARROW: ((App_GripperCloth*)example)->m_gripperVelocity = btVector3(0, 0, 0);	break;

			case 'o':
			case 'O':
				((App_GripperCloth*)example)->m_gripperFingerVelocity = btScalar(0);
				break;

			case 'p':
			case 'P':
				((App_GripperCloth*)example)->m_gripperFingerVelocity = btScalar(0);
				break;
		}
	}
	
	

	return;
}

class LessDummyGuiHelper : public DummyGUIHelper
{
	CommonGraphicsApp* m_app;

public:
	virtual CommonGraphicsApp* getAppInterface()
	{
		return m_app;
	}

	LessDummyGuiHelper(CommonGraphicsApp* app)
		: m_app(app)
	{
	}
};
int main(int argc, char* argv[])
{
	app = new SimpleOpenGL3App("Bullet Standalone Example", 1024, 768, true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);

	OpenGLGuiHelper gui(app, false);
	//LessDummyGuiHelper gui(app);
	//DummyGUIHelper gui;

	CommonExampleOptions options(&gui);

	example = App_GripperClothCreateFunc(options);
	example->processCommandLineArgs(argc, argv);

	example->initPhysics();
	example->resetCamera();

	b3Clock clock;

	app->m_window->setKeyboardCallback((b3KeyboardCallback)OnKeyboardPressed);
	// app->m_window->setKeyboardCallback((b3KeyboardCallback)(example->keyboardCallback));

	do
	{
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

		btScalar dtSec = btScalar(clock.getTimeInSeconds());
		if (dtSec < 0.1)
			dtSec = 0.1;

		if(gRunSimulation)
		{
			example->stepSimulation(dtSec);
		}
		clock.reset();

		example->renderScene();

		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);

		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	example->exitPhysics();
	delete example;
	delete app;
	return 0;
}
