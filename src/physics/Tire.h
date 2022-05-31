//
// Created by Michael Dodd 2022.
//

#ifndef BULLETGAME_SRC_PHYSICS_TIRE_H_
#define BULLETGAME_SRC_PHYSICS_TIRE_H_

#include "btBulletDynamicsCommon.h"

// TODO add rotational inertia to the tire
struct Tire
{
	Tire(const btVector3& mLocalChassisConnectionPosition,
			const btVector3& mLocalRotationAxis,
			const btVector3& mLocalSuspensionDir,
			const btScalar mSuspensionLength,
			btScalar mRadius,
			btScalar mWidth,
			btScalar mFriction,
			bool isTireSteerable);
 public:
	/// distance between the chassis connection and the axle
	const btScalar m_suspensionLength;

	btTransform m_worldTransform;

	/// position where the tire suspension connects to the chassis in world space
	btVector3 m_ChassisConnectionPosition;
	/// suspension travel direction in world space. Directed from the axle to the bottom of the tire [unit length]
	btVector3 m_suspensionDir;
	/// tire axle in world space
	btVector3 m_rotationAxis;

	/// position where the tire suspension connects to the chassis in local space
	btVector3 m_localChassisConnectionPosition;
	/// suspension travel direction in local space. Directed from the axle to the bottom of the tire [unit length]
	btVector3 m_localSuspensionDir;
	/// tire axle in local space
	btVector3 m_localRotationAxis;
	/// the velocity of the tire
	btScalar m_localVelocity;

	/// position where the tire is contacting the ground in world space
	btVector3 m_groundContactPosition;
	btVector3 m_groundNormal;
	btScalar m_penetrationDepth;

	/// current rotation angle of the tire (only used for tire animation -- no physics yet TODO)
	btScalar m_currentRotation{};

	btScalar m_radius{};
	btScalar m_width{};
	btScalar m_friction{};
	btScalar m_currentSuspensionLength{};
	btScalar m_engineTorque{};
	btScalar m_brakeTorque{};

	/// whether the wheel is in contact with the ground based on ray cast result
	bool m_isContactingGround{};

	/// Steering angle [rad.]
	btScalar m_steeringAngle{};

	bool isSteerable;
};

#endif //BULLETGAME_SRC_PHYSICS_TIRE_H_
