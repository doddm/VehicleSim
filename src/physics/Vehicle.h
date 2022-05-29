#ifndef BULLETGAME_VEHICLE_H
#define BULLETGAME_VEHICLE_H

#include "btBulletDynamicsCommon.h"
#include "Tire.h"

class Vehicle : public btActionInterface
{
 public:
	Vehicle(btRigidBody* pBody);
	void Update();
	void UpdateFriction();
	void UpdateSuspension();
	Tire& AddTire(const btVector3& position, const btVector3& rotationAxis, const btVector3& suspensionDir, btScalar friction,
			btScalar width, btScalar radius);
	int GetNumTires() const;
	const Tire& GetTire(int tireIndex) const;
	void updateTireWorldPositionRotation(Tire& tire);
	void updateTireTransform(int tireIndex);
	void SetTireTorque();
	void SetBrake();
	void SetAccelerator();
	void SetSteering();

	~Vehicle() override;

	void updateAction(btCollisionWorld* collisionWorld, btScalar step) override
	{
		(void)collisionWorld;
//		updateVehicle(step);
	}

	void debugDraw(btIDebugDraw* debugDrawer) override;

private:
	btRigidBody* m_chassisRigidBody;
	btAlignedObjectArray<Tire> m_tires;
};

#endif // BULLETGAME_VEHICLE_H
