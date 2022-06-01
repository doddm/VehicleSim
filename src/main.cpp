#include <iostream>
#include "Utils/b3Clock.h"
#include "CommonInterfaces/CommonExampleInterface.h"
#include "VehicleSim.h"

#include "../external/bullet/examples/OpenGLWindow/SimpleOpenGL3App.h"
#include "../external/bullet/examples/OpenGLWindow/OpenGLInclude.h"

#include "graphics/OpenGLGuiHelper.h"
#include "util/DebugUtil.h"

static CommonWindowInterface* s_window = 0;
CommonExampleInterface* vehicleSim;

static bool renderVisualGeometry = true;
bool visualWireframe = false;
int gDebugDrawFlags = 0;

b3KeyboardCallback prevKeyboardCallback = 0;

void MyKeyboardCallback(int key, int state)
{
	bool handled = false;

	if (!handled && vehicleSim)
	{
		vehicleSim->keyboardCallback(key, state);
	}

	if (key == 'w' && state)
	{
		visualWireframe = !visualWireframe;
		gDebugDrawFlags ^= btIDebugDraw::DBG_DrawWireframe;
	}

	if (key == B3G_ESCAPE && s_window)
	{
		s_window->setRequestExit();
	}

	if (prevKeyboardCallback)
	{
		prevKeyboardCallback(key, state);
	}
}

int main(int argc, char* argv[])
{
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Vehicle Simulation", 1920, 1080, true);

	app->m_window->setKeyboardCallback(MyKeyboardCallback);
	s_window = app->m_window;

	std::cout << "Hello World" << std::endl;

	OpenGLGuiHelper gui(app, false);

	CommonExampleOptions options(&gui);

	vehicleSim = VehicleSimCreateFunc(options);
	vehicleSim->initPhysics();
	vehicleSim->resetCamera();

	b3Clock clock;

	while (!app->m_window->requestedExit())
	{
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

		vehicleSim->stepSimulation(0.1);
		// this updates the camera target position to track the vehicle
		vehicleSim->updateGraphics();

		if (renderVisualGeometry && ((gDebugDrawFlags & btIDebugDraw::DBG_DrawWireframe) == 0))
		{
			if (visualWireframe)
			{
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			}
			vehicleSim->renderScene();
		}
		else
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			vehicleSim->physicsDebugDraw(gDebugDrawFlags);
		}

		app->swapBuffer();
	}

	vehicleSim->exitPhysics();
	delete vehicleSim;
	delete app;

	return 0;
}