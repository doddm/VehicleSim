#include <iostream>
#include "Utils/b3Clock.h"
#include "CommonInterfaces/CommonExampleInterface.h"
#include "VehicleSim.h"

#include "../external/bullet/examples/OpenGLWindow/SimpleOpenGL3App.h"
#include "../external/bullet/examples/OpenGLWindow/OpenGLInclude.h"

#include "graphics/OpenGLGuiHelper.h"

static const double k_microsecondsPerSecond = 1e6;
static CommonWindowInterface* s_window = 0;
static double gMinUpdateTimeMicroSecs = 1000.;
CommonExampleInterface* vehicleSim;

static bool renderVisualGeometry = true;
bool visualWireframe = false;
bool isSimPaused = false;
bool singleStepSimulation = false;
int gDebugDrawFlags = 0;

b3KeyboardCallback prevKeyboardCallback = 0;

void Update(SimpleOpenGL3App* app, float deltaTime);

void MyKeyboardCallback(int key, int state)
{
	bool handled = false;

	if (!handled && vehicleSim)
	{
		vehicleSim->keyboardCallback(key, state);
	}

	if(key == 'p' && state)
	{
		isSimPaused = !isSimPaused;
	}

	if (key == 'o' && state)
	{
		singleStepSimulation = true;
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

	OpenGLGuiHelper gui(app, false);

	CommonExampleOptions options(&gui);

	vehicleSim = VehicleSimCreateFunc(options);
	vehicleSim->initPhysics();
	vehicleSim->resetCamera();

	b3Clock clock;

	while (!app->m_window->requestedExit())
	{
		float deltaTimeInSeconds = clock.getTimeMicroseconds() / k_microsecondsPerSecond;
		if (deltaTimeInSeconds > 0.1)
		{
			deltaTimeInSeconds = 0.1;
		}
		if (deltaTimeInSeconds < (gMinUpdateTimeMicroSecs / k_microsecondsPerSecond))
		{
			b3Clock::usleep(gMinUpdateTimeMicroSecs / 10.);
		}
		else
		{
			clock.reset();
			Update(app, deltaTimeInSeconds);
		}
	}

	vehicleSim->exitPhysics();
	delete vehicleSim;
	delete app;

	return 0;
}

void Update(SimpleOpenGL3App* app, float deltaTime)
{
	app->m_instancingRenderer->init();
	app->m_instancingRenderer->updateCamera(app->getUpAxis());

	if(!isSimPaused || singleStepSimulation)
	{
		vehicleSim->stepSimulation(deltaTime);
	}
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

	singleStepSimulation = false;

	app->swapBuffer();
}
