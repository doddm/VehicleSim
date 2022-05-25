//
// Created by mchld on 5/23/2022.
//

#include "Vehicle.h"

void Vehicle::Update()
{

}
void Vehicle::UpdateFriction()
{

}

Tire Vehicle::AddTire(const btVector3& position, const btVector3& rotationAxis, btScalar radius, btScalar width, btScalar friction)
{
	Tire tireToAdd(position, rotationAxis, radius, width, friction);
	m_tires.push_back(tireToAdd);
	return tireToAdd;
}