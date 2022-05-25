//
// Created by mchld on 5/23/2022.
//

#include "Tire.h"

Tire::Tire(const btVector3& mLocalPosition,
	const btVector3& mLocalRotationAxis,
	btScalar mRadius,
	btScalar mWidth,
	btScalar mFriction)
	: m_localPosition(mLocalPosition),
	  m_localRotationAxis(mLocalRotationAxis),
	  m_radius(mRadius),
	  m_width(mWidth),
	  m_friction(mFriction)
{
}
