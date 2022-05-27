#include "Vehicle.h"

void Vehicle::Update()
{

}

void Vehicle::UpdateFriction()
{

}

int Vehicle::GetNumTires() const
{
	return m_tires.size();
}

Tire Vehicle::AddTire(const btVector3& position, const btVector3& rotationAxis, btScalar radius, btScalar width, btScalar friction)
{
	Tire tireToAdd(position, rotationAxis, radius, width, friction);
	m_tires.push_back(tireToAdd);
	return tireToAdd;
}
Vehicle::Vehicle()
{
}

Vehicle::~Vehicle()
{
}

void Vehicle::debugDraw(btIDebugDraw* debugDrawer)
{
}
