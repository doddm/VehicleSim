//
// Created by mchld on 5/23/2022.
//

#ifndef BULLETGAME_VEHICLE_H
#define BULLETGAME_VEHICLE_H

#include "BulletDynamics/Dynamics/btActionInterface.h"

class Vehicle : public btActionInterface
{
 public:
	void Update();
	void UpdateFriction();
	void UpdateSuspension();
	void AddWheel();
	void SetWheelTorque();
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
