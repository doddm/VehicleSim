/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
 * This software has been modified by Michael Dodd (2022)
 */

#ifndef COMMON_EXAMPLE_INTERFACE_H
#define COMMON_EXAMPLE_INTERFACE_H

struct CommandProcessorCreationInterface
{
	virtual ~CommandProcessorCreationInterface() {}
	virtual class CommandProcessorInterface* createCommandProcessor() = 0;
	virtual void deleteCommandProcessor(CommandProcessorInterface*) = 0;
};

struct CommonExampleOptions
{
	struct GUIHelperInterface* m_guiHelper;

	//Those are optional, some examples will use them others don't. Each example should work with them being 0.
	int m_option;
	const char* m_fileName;
	class SharedMemoryInterface* m_sharedMem;
	CommandProcessorCreationInterface* m_commandProcessorCreation;
	bool m_skipGraphicsUpdate;

	CommonExampleOptions(struct GUIHelperInterface* helper, int option = 0)
		: m_guiHelper(helper),
		  m_option(option),
		  m_fileName(0),
		  m_sharedMem(0),
		  m_commandProcessorCreation(0),
		  m_skipGraphicsUpdate(false)
	{
	}
};

class CommonExampleInterface
{
public:
	typedef class CommonExampleInterface*(CreateFunc)(CommonExampleOptions& options);

	virtual ~CommonExampleInterface()
	{
	}

	virtual void initPhysics() = 0;
	virtual void exitPhysics() = 0;
	virtual void updateGraphics() {}
	virtual void stepSimulation(float deltaTime) = 0;
	virtual void renderScene() = 0;
	virtual void physicsDebugDraw(int debugFlags) = 0;  //for now we reuse the flags in Bullet/src/LinearMath/btIDebugDraw.h
	//reset camera is only called when switching demo. this way you can restart (initPhysics) and watch in a specific location easier
	virtual void resetCamera(){};
	virtual bool mouseMoveCallback(float x, float y) = 0;
	virtual bool mouseButtonCallback(int button, int state, float x, float y) = 0;
	virtual bool keyboardCallback(int key, int state) = 0;

    virtual void vrGenericTrackerMoveCallback(int controllerId, float pos[4], float orientation[4]) {}

	virtual void processCommandLineArgs(int argc, char* argv[]){};
};

class ExampleEntries
{
public:
	virtual ~ExampleEntries() {}

	virtual void initExampleEntries() = 0;

	virtual void initOpenCLExampleEntries() = 0;

	virtual int getNumRegisteredExamples() = 0;

	virtual CommonExampleInterface::CreateFunc* getExampleCreateFunc(int index) = 0;

	virtual const char* getExampleName(int index) = 0;

	virtual const char* getExampleDescription(int index) = 0;

	virtual int getExampleOption(int index) = 0;
};

CommonExampleInterface* StandaloneExampleCreateFunc(CommonExampleOptions& options);

#ifdef B3_USE_STANDALONE_EXAMPLE
#define B3_STANDALONE_EXAMPLE(ExampleFunc)                                             \
	CommonExampleInterface* StandaloneExampleCreateFunc(CommonExampleOptions& options) \
	{                                                                                  \
		return ExampleFunc(options);                                                   \
	}
#else  //B3_USE_STANDALONE_EXAMPLE
#define B3_STANDALONE_EXAMPLE(ExampleFunc)
#endif  //B3_USE_STANDALONE_EXAMPLE

#endif  //COMMON_EXAMPLE_INTERFACE_H
