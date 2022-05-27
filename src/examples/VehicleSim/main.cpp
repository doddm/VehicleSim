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

#include "../CommonExampleInterface.h"
#include "../../graphics/CommonGUIHelperInterface.h"
#include "../../../external/bullet/examples/Utils/b3Clock.h"

#include "../../../external/bullet/examples/OpenGLWindow/SimpleOpenGL3App.h"
#include "../../graphics/OpenGLGuiHelper.h"
#include "LinearMath/btIDebugDraw.h"
#include "VehicleSim.h"

static CommonGraphicsApp* s_app = 0;
static CommonWindowInterface* s_window = 0;

CommonExampleInterface* vehicleSim;
int gSharedMemoryKey = -1;
static bool gEnableDefaultKeyboardShortcuts = true;
static bool gEnableDefaultMousePicking = true;
bool renderGui = true;
bool gDisableDemoSelection = false;
bool visualWireframe = false;
bool drawGrid = false;
static bool renderVisualGeometry = true;
static bool renderGrid = true;
static bool gEnableRenderLoop = true;
static bool pauseSimulation = false;
static bool singleStepSimulation = false;

int gDebugDrawFlags = 0;

#ifndef USE_OPENGL3
extern bool useShadowMap;
#endif

b3KeyboardCallback prevKeyboardCallback = 0;

void MyKeyboardCallback(int key, int state)
{
    //b3Printf("key=%d, state=%d", key, state);
    bool handled = false;

    if (!handled && vehicleSim)
    {
        handled = vehicleSim->keyboardCallback(key, state);
    }

    if (gEnableDefaultKeyboardShortcuts)
    {
        if (key == 'a' && state)
        {
            gDebugDrawFlags ^= btIDebugDraw::DBG_DrawAabb;
        }
        if (key == 'c' && state)
        {
            gDebugDrawFlags ^= btIDebugDraw::DBG_DrawContactPoints;
        }
        if (key == 'j' && state)
        {
            gDebugDrawFlags ^= btIDebugDraw::DBG_DrawFrames;
        }

        if (key == 'k' && state)
        {
            gDebugDrawFlags ^= btIDebugDraw::DBG_DrawConstraints;
        }

        if (key == 'l' && state)
        {
            gDebugDrawFlags ^= btIDebugDraw::DBG_DrawConstraintLimits;
        }
        if (key == 'w' && state)
        {
            visualWireframe = !visualWireframe;
            gDebugDrawFlags ^= btIDebugDraw::DBG_DrawWireframe;
        }

        if (key == 'v' && state)
        {
            renderVisualGeometry = !renderVisualGeometry;
        }
        if (key == 'g' && state)
        {
            renderGrid = !renderGrid;
            renderGui = !renderGui;
        }

        if (key == 'i' && state)
        {
            pauseSimulation = !pauseSimulation;
        }
        if (key == 'o' && state)
        {
            singleStepSimulation = true;
        }

#ifndef NO_OPENGL3
        if (key == 's' && state)
        {
            useShadowMap = !useShadowMap;
        }
#endif

    }
    if (key == B3G_ESCAPE && s_window)
    {
        s_window->setRequestExit();
    }

    if (prevKeyboardCallback)
        prevKeyboardCallback(key, state);
}

#include "../../../external/bullet/examples/SharedMemory/SharedMemoryPublic.h"

void OpenGLExampleBrowserVisualizerFlagCallback(int flag, bool enable)
{
    if (flag == COV_ENABLE_Y_AXIS_UP)
    {
        //either Y = up or Z
        int upAxis = enable ? 1 : 2;
        s_app->setUpAxis(upAxis);
    }

    if (flag == COV_ENABLE_RENDERING)
    {
        gEnableRenderLoop = (enable != 0);
    }

    if (flag == COV_ENABLE_SINGLE_STEP_RENDERING)
    {
        if (enable)
        {
            gEnableRenderLoop = false;
            singleStepSimulation = true;
        }
        else
        {
            gEnableRenderLoop = true;
            singleStepSimulation = false;
        }
    }

    if (flag == COV_ENABLE_SHADOWS)
    {
        useShadowMap = enable;
    }
    if (flag == COV_ENABLE_GUI)
    {
        renderGui = enable;
        renderGrid = enable;
    }

    if (flag == COV_ENABLE_KEYBOARD_SHORTCUTS)
    {
        gEnableDefaultKeyboardShortcuts = enable;
    }
    if (flag == COV_ENABLE_MOUSE_PICKING)
    {
        gEnableDefaultMousePicking = enable;
    }

    if (flag == COV_ENABLE_WIREFRAME)
    {
        visualWireframe = enable;
        if (visualWireframe)
        {
            gDebugDrawFlags |= btIDebugDraw::DBG_DrawWireframe;
        }
        else
        {
            gDebugDrawFlags &= ~btIDebugDraw::DBG_DrawWireframe;
        }
    }
}

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove(float x, float y)
{
	bool handled = false;
	handled = vehicleSim->mouseMoveCallback(x, y);
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

	handled = vehicleSim->mouseButtonCallback(button, state, x, y);
	if (!handled)
	{
		if (prevMouseButtonCallback)
			prevMouseButtonCallback(button, state, x, y);
	}
}

int main(int argc, char* argv[])
{
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Vehicle Sim", 1920, 1080, true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);
    app->m_window->setKeyboardCallback(MyKeyboardCallback);
    s_window = app->m_window;
    s_app = app;

	OpenGLGuiHelper gui(app, false);
    gui.setVisualizerFlagCallback(OpenGLExampleBrowserVisualizerFlagCallback);

	CommonExampleOptions options(&gui);

	vehicleSim = VehicleSimCreateFunc(options);
	vehicleSim->processCommandLineArgs(argc, argv);

	vehicleSim->initPhysics();
	vehicleSim->resetCamera();

	b3Clock clock;

	do
	{
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

        //TODO (mdodd) move this into its own routine
        char bla[1024];
        float yellow[4] = {0, 0, 0, 1};

		btScalar dtSec = btScalar(clock.getTimeInSeconds());

        sprintf(bla, "FPS: %g", 1.0/dtSec);
        app->drawText(bla, 10, 10, 1, yellow);

        if (dtSec > 0.1)
        {
            dtSec = 0.1;
        }

		vehicleSim->stepSimulation(dtSec);
		clock.reset();

		vehicleSim->renderScene();

		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
        if(drawGrid)
        {
            app->drawGrid(dg);
        }

		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	vehicleSim->exitPhysics();
	delete vehicleSim;
	delete app;
	return 0;
}
