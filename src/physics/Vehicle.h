#ifndef BULLETGAME_VEHICLE_H
#define BULLETGAME_VEHICLE_H

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "Tire.h"

class Vehicle : public btActionInterface
{
 public:
	btAlignedObjectArray<Tire> m_tires;

	Vehicle();
	void Update();
	void UpdateFriction();
	void UpdateSuspension();
	Tire AddTire(const btVector3& position, const btVector3& rotationAxis, btScalar radius, btScalar width, btScalar friction);
	int GetNumTires() const;
	void SetTireTorque();
	void SetBrake();
	void SetAccelerator();
	void SetSteering();

	virtual ~Vehicle();

	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step)
	{
		(void)collisionWorld;
//		updateVehicle(step);
	}

	virtual void debugDraw(btIDebugDraw* debugDrawer);
};

#endif // BULLETGAME_VEHICLE_H
