#include "Vehicle.h"

Vehicle::Vehicle(btRigidBody* pBody)
{
	m_chassisRigidBody = pBody;
}

void Vehicle::Update()
{
	return;
}

void Vehicle::UpdateFriction()
{
	return;
}

int Vehicle::GetNumTires() const
{
	return m_tires.size();
}

Tire& Vehicle::AddTire(const btVector3& position, const btVector3& rotationAxis, const btVector3& suspensionDir,
		btScalar friction, btScalar width, btScalar radius)
{
	Tire tireToAdd(position, rotationAxis, suspensionDir, radius, width, friction);
	m_tires.push_back(tireToAdd);
	updateTireWorldPositionRotation(tireToAdd);
	updateTireTransform(GetNumTires() - 1);
	return tireToAdd;
}

const Tire& Vehicle::GetTire(int tireIndex) const
{
	return m_tires[tireIndex];
}

void Vehicle::updateTireWorldPositionRotation(Tire& tire)
{
	btTransform chassisTransform = m_chassisRigidBody->getCenterOfMassTransform();
	tire.m_position = chassisTransform(tire.m_localPosition);
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
	tire.m_worldTransform.setOrigin(
			tire.m_position + tire.m_suspensionDir * tire.m_suspensionLength);
}

Vehicle::~Vehicle()
{
}

void Vehicle::debugDraw(btIDebugDraw* debugDrawer)
{
}
