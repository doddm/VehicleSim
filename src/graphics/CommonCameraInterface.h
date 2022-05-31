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

#ifndef COMMON_CAMERA_INTERFACE_H
#define COMMON_CAMERA_INTERFACE_H

struct CommonCameraInterface
{
	virtual ~CommonCameraInterface() {}
	virtual void getCameraProjectionMatrix(float m[16]) const = 0;
	virtual void getCameraViewMatrix(float m[16]) const = 0;

	virtual void setVRCamera(const float viewMat[16], const float projectionMatrix[16]) = 0;
	virtual void disableVRCamera() = 0;
	virtual bool isVRCamera() const = 0;
	virtual void setVRCameraOffsetTransform(const float offset[16]) = 0;

	virtual void getCameraTargetPosition(float pos[3]) const = 0;
	virtual void getCameraPosition(float pos[3]) const = 0;

	virtual void getCameraTargetPosition(double pos[3]) const = 0;
	virtual void getCameraPosition(double pos[3]) const = 0;

	virtual void setCameraTargetPosition(float x, float y, float z) = 0;
	virtual void setCameraDistance(float dist) = 0;
	virtual float getCameraDistance() const = 0;

	virtual void setCameraUpVector(float x, float y, float z) = 0;
	virtual void getCameraUpVector(float up[3]) const = 0;
	virtual void getCameraForwardVector(float fwd[3]) const = 0;

	///the setCameraUpAxis will call the 'setCameraUpVector' and 'setCameraForwardVector'
	virtual void setCameraUpAxis(int axis) = 0;
	virtual int getCameraUpAxis() const = 0;

	virtual void setCameraYaw(float yaw) = 0;
	virtual float getCameraYaw() const = 0;

	virtual void setCameraPitch(float pitch) = 0;
	virtual float getCameraPitch() const = 0;

	virtual void setAspectRatio(float ratio) = 0;
	virtual float getAspectRatio() const = 0;

	virtual float getCameraFrustumFar() const = 0;
	virtual float getCameraFrustumNear() const = 0;
};

#endif  //COMMON_CAMERA_INTERFACE_H
