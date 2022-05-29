#ifndef BULLETGAME_SRC_PHYSICS_TIRE_H_
#define BULLETGAME_SRC_PHYSICS_TIRE_H_

#include "btBulletDynamicsCommon.h"

// TODO add rotational inertia to the tire
struct Tire
{
	Tire(const btVector3& mLocalPosition, const btVector3& mLocalRotationAxis,
			const btVector3& mLocalSuspensionDir, btScalar mRadius, btScalar mWidth, btScalar mFriction);
 public:
	btTransform m_worldTransform;

	/// the tire position in world space
	btVector3 m_position;
	/// the suspension travel direction in world space. Directed from the axle towards the ground
	btVector3 m_suspensionDir;
	/// the tire axle in world space
	btVector3 m_rotationAxis;

	/// the tire position in local space
	btVector3 m_localPosition;
	/// the suspension travel direction in local space. Directed from the axle towards the ground
	btVector3 m_localSuspensionDir;
	/// the tire axle in local space
	btVector3 m_localRotationAxis;
	/// the current rotation angle of the tire (only used for tire animation -- no physics yet TODO)
	btScalar m_currentRotation{};

	btScalar m_radius{};
	btScalar m_width{};
	btScalar m_friction{};
	btScalar m_suspensionStiffness{};
	btScalar m_suspensionDamping{};
	btScalar m_suspensionLength{};
	btScalar m_engineTorque{};
	btScalar m_brakeTorque{};

	/// whether the wheel is in contact with the ground based on ray cast result
	bool m_isContactingGround{};

	/// Steering angle [rad.]
	btScalar m_steeringAngle{};
};

#endif //BULLETGAME_SRC_PHYSICS_TIRE_H_
