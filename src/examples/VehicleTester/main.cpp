#include <iostream>
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "Utils/b3Clock.h"
#include "physics/Vehicle.h"
//#include "physics/Vehicle.h"

static CommonWindowInterface* s_window = 0;

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

int main(int argc, char* argv[])
{
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Vehicle Sim", 1920, 1080, true);
	app->m_window->setKeyboardCallback(MyKeyboardCallback);
	s_window = app->m_window;

	std::cout << "Hello World" << std::endl;

	Vehicle* truck;
	truck = new Vehicle();
	btVector3 tirePosition {1, 2, 3};
	truck->AddTire(tirePosition, tirePosition, 4.0, 5.0, 86.0);
	truck->AddTire(tirePosition, tirePosition, 4.0, 5.0, 2005.0);

	std::cout << truck->GetNumTires() << std::endl;
	std::cout << truck->m_tires[0].m_friction << std::endl;

	b3Clock clock;

	while (!app->m_window->requestedExit())
	{
//		clock.getTimeMicroseconds();
//		clock.reset();
		app->swapBuffer();
	}

	return 0;
}