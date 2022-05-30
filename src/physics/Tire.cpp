#include "Tire.h"

Tire::Tire(const btVector3& mLocalPosition, const btVector3& mLocalRotationAxis,
		const btVector3& mLocalSuspensionDir, btScalar mRadius, btScalar mWidth, btScalar mFriction)
		:m_localPosition(mLocalPosition),
		 m_localRotationAxis(mLocalRotationAxis),
		 m_localSuspensionDir(mLocalSuspensionDir),
		 m_radius(mRadius),
		 m_width(mWidth),
		 m_friction(mFriction)
{
	m_currentRotation = 0;
}
