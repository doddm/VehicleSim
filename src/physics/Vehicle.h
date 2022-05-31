#ifndef BULLETGAME_VEHICLE_H
#define BULLETGAME_VEHICLE_H

#include "btBulletDynamicsCommon.h"
#include "Tire.h"
#include "Raycast.h"

class Vehicle : public btActionInterface
{
 public:
	const int numWheels = 4;

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
	void setTireWorldTransform(int tireIndex);
	// TODO move this and the Raycast member to the Tire class
	bool castRay(Tire& tire);
	void setTireTorque();
	void setBrake(btScalar brakeForce);
	void setAccelerator(btScalar engineForce);
	void setSteering(btScalar angle);
	void setSuspensionStiffness(btScalar stiffness);
	btScalar getSuspensionStiffness();
	void setSuspensionDamping(btScalar damping);
	btScalar getSuspensionDamping();

	~Vehicle() override;

	///btActionInterface implementation
	void updateAction(btCollisionWorld* collisionWorld, btScalar step) override
	{
		(void)collisionWorld;
		update(step);
	}

	void debugDraw(btIDebugDraw* debugDrawer) override;

private:
	Raycast* m_raycast;
	btRigidBody* m_chassisRigidBody;
	btAlignedObjectArray<Tire> m_tires;
	btScalar m_suspensionStiffness;
	btScalar m_suspensionDamping;
};

#endif // BULLETGAME_VEHICLE_H
