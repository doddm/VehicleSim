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
	const btScalar m_suspensionLength {0};

	btTransform m_worldTransform {};

	/// position where the tire suspension connects to the chassis in world space
	btVector3 m_chassisConnectionPosWorld {};
	/// suspension travel direction in world space. Directed from the axle to the bottom of the tire [unit length]
	btVector3 m_suspensionDirWorld {};
	/// tire axle in world space
	btVector3 m_rotationAxisWorld {};

	/// position where the tire suspension connects to the chassis in local space
	btVector3 m_chassisConnectionPosLocal {};
	/// suspension travel direction in local space. Directed from the axle to the bottom of the tire [unit length]
	btVector3 m_suspensionDirLocal {};
	/// tire axle in local space
	btVector3 m_rotationAxisLocal {};
	/// the velocity of the tire in local space [m/s]
	btScalar m_velocityLocal {0};

	/// position where the tire is contacting the ground in world space
	btVector3 m_groundContactPosition {};
	btVector3 m_groundNormal {};
	btScalar m_penetrationDepth {0};

	/// tire friction data
	btVector3 m_groundContactVelWorld {};
	btVector3 m_groundContactFwdWorld {};
	btVector3 m_latFrictionWorld {};
	btVector3 m_lonFrictionWorld {};
	btScalar m_friction {0};

	/// current rotation angle of the tire (only used for tire animation -- no physics yet TODO)
	btScalar m_currentRotation {0};
	btScalar m_deltaRotation {0};
	btScalar m_radius {0};
	btScalar m_width {0};
	btScalar m_currentSuspensionLength {0};
	btScalar m_engineForce {0};
	btScalar m_brakeForce {0};

	/// whether the wheel is in contact with the ground based on ray cast result
	bool m_isContactingGround {false};

	/// Steering angle [rad.]
	btScalar m_steeringAngle {0};

	bool isSteerable {false};
};

#endif //BULLETGAME_SRC_PHYSICS_TIRE_H_
