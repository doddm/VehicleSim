//
// Created by mchld on 5/23/2022.
//

#ifndef BULLETGAME_SRC_PHYSICS_TIRE_H_
#define BULLETGAME_SRC_PHYSICS_TIRE_H_

#include "btBulletDynamicsCommon.h"

struct Tire
{
	Tire(const btVector3& mLocalPosition,
		const btVector3& mLocalRotationAxis,
		btScalar mRadius,
		btScalar mWidth,
		btScalar mFriction);
 public:
	btVector3 m_localPosition;
	btVector3 m_localRotationAxis;
	btScalar m_radius{};
	btScalar m_width{};
	btScalar m_friction{};
	btScalar m_suspensionStiffness{};
	btScalar m_suspensionDamping{};
	btScalar m_engineTorque{};
	btScalar m_brakeTorque{};
};

#endif //BULLETGAME_SRC_PHYSICS_TIRE_H_
