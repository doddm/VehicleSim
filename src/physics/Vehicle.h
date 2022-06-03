#ifndef BULLETGAME_VEHICLE_H
#define BULLETGAME_VEHICLE_H

#include "btBulletDynamicsCommon.h"
#include "Tire.h"
#include "Raycast.h"

class Vehicle : public btActionInterface
{
 public:
	const int m_numWheels = 4;

	explicit Vehicle(btRigidBody* pBody, Raycast* pRaycast);
	void update(btScalar step);
	void updateTireFriction(btScalar step);
	void updateSuspension(btScalar step);
	const btRigidBody* getRigidBody() const;
	const btTransform& getChassisWorldTransform() const;
	Tire& addTire(const btVector3& position, const btVector3& rotationAxis, const btVector3& suspensionDir, btScalar suspensionLength, btScalar friction,
			btScalar width, btScalar radius, bool isSteerable);
	int getNumTires() const;
	const Tire& getTire(int tireIndex) const;
	void updateTireWorldPositionRotation(Tire& tire);
	void updateTireWorldTransform(int tireIndex);
	// TODO move this and the Raycast member to the Tire class
	bool castRay(Tire& tire);
	void setBrake(btScalar brakeForce);
	void setAccelerator(btScalar engineForce);
	void setSteering(btScalar angle, btScalar increment);
	void setSuspensionStiffness(btScalar stiffness);
	void setSuspensionDamping(btScalar damping);

	~Vehicle() override;

	///btActionInterface implementation
	void updateAction(btCollisionWorld* collisionWorld, btScalar step) override
	{
		(void)collisionWorld;
		update(step);
	}

	void debugDraw(btIDebugDraw* debugDrawer) override;
	void setTireFrictionActive(bool isActive);

private:
	Raycast* m_raycast;
	btRigidBody* m_chassisRigidBody;
	btAlignedObjectArray<Tire> m_tires;
	btScalar m_suspensionStiffness;
	btScalar m_suspensionDamping;
	bool m_isTireFrictionActive;
};

#endif // BULLETGAME_VEHICLE_H
