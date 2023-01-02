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

#ifndef B3G_WINDOW_INTERFACE_H
#define B3G_WINDOW_INTERFACE_H

#include "input/CommonCallbacks.h"

struct b3gWindowConstructionInfo
{
	int m_width;
	int m_height;
	bool m_fullscreen;
	int m_colorBitsPerPixel;
	void* m_windowHandle;
	const char* m_title;
	int m_openglVersion;
	int m_renderDevice;

	b3gWindowConstructionInfo(int width = 1024, int height = 768)
		: m_width(width),
		  m_height(height),
		  m_fullscreen(false),
		  m_colorBitsPerPixel(32),
		  m_windowHandle(0),
		  m_title("title"),
		  m_openglVersion(3),
		  m_renderDevice(-1)
	{
	}
};

class CommonWindowInterface
{
public:
	virtual ~CommonWindowInterface()
	{
	}

	virtual void createDefaultWindow(int width, int height, const char* title)
	{
		b3gWindowConstructionInfo ci(width, height);
		ci.m_title = title;
		createWindow(ci);
	}

	virtual void createWindow(const b3gWindowConstructionInfo& ci) = 0;

	virtual void closeWindow() = 0;

	virtual void runMainLoop() = 0;
	virtual float getTimeInSeconds() = 0;

	virtual bool requestedExit() const = 0;
	virtual void setRequestExit() = 0;

	virtual void startRendering() = 0;

	virtual void endRendering() = 0;

	virtual bool isModifierKeyPressed(int key) = 0;

	virtual void setMouseMoveCallback(b3MouseMoveCallback mouseCallback) = 0;
	virtual b3MouseMoveCallback getMouseMoveCallback() = 0;

	virtual void setMouseButtonCallback(b3MouseButtonCallback mouseCallback) = 0;
	virtual b3MouseButtonCallback getMouseButtonCallback() = 0;

	virtual void setResizeCallback(b3ResizeCallback resizeCallback) = 0;
	virtual b3ResizeCallback getResizeCallback() = 0;

	virtual void setWheelCallback(b3WheelCallback wheelCallback) = 0;
	virtual b3WheelCallback getWheelCallback() = 0;

	virtual void setKeyboardCallback(b3KeyboardCallback keyboardCallback) = 0;
	virtual b3KeyboardCallback getKeyboardCallback() = 0;

	virtual void setRenderCallback(b3RenderCallback renderCallback) = 0;

	virtual void setWindowTitle(const char* title) = 0;

	virtual float getRetinaScale() const = 0;
	virtual void setAllowRetina(bool allow) = 0;

	virtual int getWidth() const = 0;
	virtual int getHeight() const = 0;

	virtual int fileOpenDialog(char* fileName, int maxFileNameLength) = 0;
};

#endif  //B3G_WINDOW_INTERFACE_H
