#include "Tire.h"

Tire::Tire(const btVector3& mLocalChassisConnectionPosition,
	const btVector3& mLocalRotationAxis,
	const btVector3& mLocalSuspensionDir,
	const btScalar mSuspensionLength,
	btScalar mRadius,
	btScalar mWidth,
	btScalar mFriction,
	bool isTireSteerable)
		: m_localChassisConnectionPosition(mLocalChassisConnectionPosition),
		  m_localRotationAxis(mLocalRotationAxis),
		  m_localSuspensionDir(mLocalSuspensionDir),
		  m_radius(mRadius),
		  m_width(mWidth),
		  m_friction(mFriction), m_suspensionLength(mSuspensionLength), isSteerable(isTireSteerable)
{
	m_currentRotation = 0;
}
