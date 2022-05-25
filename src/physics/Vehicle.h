//
// Created by mchld on 5/23/2022.
//

#ifndef BULLETGAME_VEHICLE_H
#define BULLETGAME_VEHICLE_H

#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "Tire.h"

class Vehicle : public btActionInterface
{
 public:
	btAlignedObjectArray<Tire> m_tires;

	void Update();
	void UpdateFriction();
	void UpdateSuspension();
	Tire AddTire(const btVector3& position, const btVector3& rotationAxis, btScalar radius, btScalar width, btScalar friction);
	void GetNumTires();
	void SetTireTorque();
	void SetBrake();
	void SetAccelerator();
	void SetSteering();

	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step)
	{
		(void)collisionWorld;
//		updateVehicle(step);
	}
};

#endif // BULLETGAME_VEHICLE_H
