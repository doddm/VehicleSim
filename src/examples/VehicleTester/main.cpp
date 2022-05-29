#include <iostream>
//#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "Utils/b3Clock.h"
#include "physics/Vehicle.h"
#include "CommonInterfaces/CommonExampleInterface.h"
//#include "ExampleBrowser/OpenGLGuiHelper.h"
#include "VehicleSim.h"

#include "../../../external/bullet/examples/OpenGLWindow/SimpleOpenGL3App.h"
#include "../../graphics/OpenGLGuiHelper.h"

static CommonWindowInterface* s_window = 0;
CommonExampleInterface* vehicleSim;

b3KeyboardCallback prevKeyboardCallback = 0;

void MyKeyboardCallback(int key, int state)
{
	//b3Printf("key=%d, state=%d", key, state);
	bool handled = false;

//	if (!handled && vehicleSim)
//	{
//		handled = vehicleSim->keyboardCallback(key, state);
//	}

	if (key == B3G_ESCAPE && s_window)
	{
		s_window->setRequestExit();
	}

	if (prevKeyboardCallback)
	{
		prevKeyboardCallback(key, state);
	}
}

#include "../../../external/bullet/examples/SharedMemory/SharedMemoryPublic.h"

void OpenGLExampleBrowserVisualizerFlagCallback(int flag, bool enable)
{
}

int main(int argc, char* argv[])
{
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Vehicle Sim", 1920, 1080, true);
	app->m_window->setKeyboardCallback(MyKeyboardCallback);
	s_window = app->m_window;

	std::cout << "Hello World" << std::endl;

	OpenGLGuiHelper gui(app, false);
	gui.setVisualizerFlagCallback(OpenGLExampleBrowserVisualizerFlagCallback);

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
		vehicleSim->renderScene();

		app->swapBuffer();
	}

	vehicleSim->exitPhysics();

	return 0;
}