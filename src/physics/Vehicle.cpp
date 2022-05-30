#include <iostream>
#include "Vehicle.h"

Vehicle::Vehicle(btRigidBody* pBody, Raycast* pRaycast)
{
	m_chassisRigidBody = pBody;
	m_raycast = pRaycast;
}

void Vehicle::update()
{
	return;
}

void Vehicle::updateFriction()
{
	return;
}

const btRigidBody* Vehicle::getRigidBody() const
{
	return m_chassisRigidBody;
}

const btTransform& Vehicle::getChassisWorldTransform() const
{
	return getRigidBody()->getCenterOfMassTransform();
}

int Vehicle::getNumTires() const
{
	return m_tires.size();
}

Tire& Vehicle::addTire(const btVector3& position, const btVector3& rotationAxis, const btVector3& suspensionDir,
		btScalar friction, btScalar width, btScalar radius)
{
	Tire tireToAdd(position, rotationAxis, suspensionDir, radius, width, friction);
	m_tires.push_back(tireToAdd);
	updateTireWorldPositionRotation(tireToAdd);
	updateTireTransform(getNumTires() - 1);
	return tireToAdd;
}

const Tire& Vehicle::getTire(int tireIndex) const
{
	return m_tires[tireIndex];
}

void Vehicle::updateTireWorldPositionRotation(Tire& tire)
{
	btTransform chassisTransform = getChassisWorldTransform();
	tire.m_position = chassisTransform(tire.m_localPosition);
	std::cout << "local position = " << tire.m_localPosition.z() << std::endl;
	std::cout << "world position = " << tire.m_position.z() << std::endl;
	tire.m_rotationAxis = chassisTransform.getBasis() * tire.m_localRotationAxis;
	tire.m_suspensionDir = chassisTransform.getBasis() * tire.m_localSuspensionDir;
	return;
}

void Vehicle::updateTireTransform(int tireIndex)
{
	Tire& tire = m_tires[tireIndex];
	updateTireWorldPositionRotation(tire);
	btVector3 up = -tire.m_suspensionDir;
	const btVector3& right = tire.m_rotationAxis;
	btVector3 fwd = up.cross(right);
	fwd = fwd.normalize();

	btScalar steeringAngle = tire.m_steeringAngle;

	btQuaternion steeringOrn(up, steeringAngle);  //wheel.m_steering);
	btMatrix3x3 steeringMat(steeringOrn);

	btQuaternion rotatingOrn(right, -tire.m_currentRotation);
	btMatrix3x3 rotatingMat(rotatingOrn);

	btMatrix3x3 basis2;
	basis2[0][0] = -right[0];
	basis2[1][0] = -right[1];
	basis2[2][0] = -right[2];

	basis2[0][1] = up[0];
	basis2[1][1] = up[1];
	basis2[2][1] = up[2];

	basis2[0][2] = fwd[0];
	basis2[1][2] = fwd[1];
	basis2[2][2] = fwd[2];

	tire.m_worldTransform.setBasis(steeringMat * rotatingMat * basis2);
	tire.m_worldTransform.setOrigin(tire.m_position + tire.m_suspensionDir * tire.m_suspensionLength);
}

void Vehicle::debugDraw(btIDebugDraw* debugDrawer)
{
}
void Vehicle::updateSuspension()
{

}
void Vehicle::setTireTorque()
{

}
void Vehicle::setBrake()
{

}
void Vehicle::setSteering()
{

}
void Vehicle::setAccelerator()
{

}
Vehicle::~Vehicle()
{

}
bool Vehicle::castRay(Tire& tire)
{
	btVector3 start;
	btVector3 stop;

	start = tire.m_position;
	stop = tire.m_suspensionDir * 10;

	RaycastHit hitInfo;
	bool isHit = m_raycast->castRay(start, stop, hitInfo);

	if(isHit)
	{
		hitInfo.debugPrint();
	}

	return false;
}
