#ifndef BULLETGAME_VEHICLE_H
#define BULLETGAME_VEHICLE_H

#include "btBulletDynamicsCommon.h"
#include "Tire.h"
#include "Raycast.h"

class Vehicle : public btActionInterface
{
 public:
	explicit Vehicle(btRigidBody* pBody, Raycast* pRaycast);
	void update();
	void updateFriction();
	void updateSuspension();
	const btRigidBody* getRigidBody() const;
	const btTransform& getChassisWorldTransform() const;
	Tire& addTire(const btVector3& position, const btVector3& rotationAxis, const btVector3& suspensionDir, btScalar friction,
			btScalar width, btScalar radius);
	int getNumTires() const;
	const Tire& getTire(int tireIndex) const;
	void updateTireWorldPositionRotation(Tire& tire);
	void updateTireTransform(int tireIndex);
	// TODO move this and the Raycast member to the Tire class
	bool castRay(Tire& tire);
	void setTireTorque();
	void setBrake();
	void setAccelerator();
	void setSteering();

	~Vehicle() override;

	void updateAction(btCollisionWorld* collisionWorld, btScalar step) override
	{
		(void)collisionWorld;
//		updateVehicle(step);
	}

	void debugDraw(btIDebugDraw* debugDrawer) override;

private:
	Raycast* m_raycast;
	btRigidBody* m_chassisRigidBody;
	btAlignedObjectArray<Tire> m_tires;
};

#endif // BULLETGAME_VEHICLE_H
