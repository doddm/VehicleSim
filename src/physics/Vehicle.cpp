#include <iostream>
#include "Vehicle.h"

Vehicle::Vehicle(btRigidBody* pBody, Raycast* pRaycast)
{
	m_chassisRigidBody = pBody;
	m_raycast = pRaycast;
}

void Vehicle::update(btScalar step)
{
	///do raycast for all wheels
	for(int i = 0; i < numWheels; i++)
	{
		updateTireWorldPositionRotation(m_tires[i]);
		castRay(m_tires[i]);
	}

	updateSuspension(step);
}

void Vehicle::updateTireFriction(btScalar step)
{

}
void Vehicle::updateSuspension(btScalar step)
{
	btScalar mass = 700.0/4;
	for(int i = 0; i < numWheels; i++)
	{
		Tire& tire = m_tires[i];
		btScalar displacement = tire.m_currentSuspensionLength - tire.m_suspensionLength;
		btScalar springForceMag = displacement * tire.m_suspensionStiffness * mass;
		btVector3 springImpulse = springForceMag * step * tire.m_suspensionDir;
		btVector3 relativePosition = tire.m_groundContactPosition - m_chassisRigidBody->getCenterOfMassPosition();
//		std::cout << "Spring impulse Mag =" << springImpulse.length() << std::endl;

		m_chassisRigidBody->applyImpulse(springImpulse, relativePosition);
	}
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

Tire& Vehicle::addTire(const btVector3& position, const btVector3& rotationAxis, const btVector3& suspensionDir, const btScalar suspensionLength,
		btScalar friction, btScalar width, btScalar radius, btScalar suspensionStiffness)
{
	Tire tireToAdd(position, rotationAxis, suspensionDir, suspensionLength, radius, width, friction, suspensionStiffness);
	m_tires.push_back(tireToAdd);
	updateTireWorldPositionRotation(tireToAdd);
	setTireWorldTransform(getNumTires() - 1);
	return tireToAdd;
}

const Tire& Vehicle::getTire(int tireIndex) const
{
	return m_tires[tireIndex];
}

void Vehicle::updateTireWorldPositionRotation(Tire& tire)
{
	btTransform chassisTransform = getChassisWorldTransform();
	tire.m_ChassisConnectionPosition = chassisTransform(tire.m_localChassisConnectionPosition);
	tire.m_rotationAxis = chassisTransform.getBasis() * tire.m_localRotationAxis;
	tire.m_suspensionDir = chassisTransform.getBasis() * tire.m_localSuspensionDir;
	return;
}

void Vehicle::setTireWorldTransform(int tireIndex)
{
	Tire& tire = m_tires[tireIndex];
	updateTireWorldPositionRotation(tire);
	btVector3 up = -tire.m_suspensionDir;
	const btVector3& right = tire.m_rotationAxis;
	btVector3 fwd = up.cross(right);
	fwd = fwd.normalize();

	btQuaternion steeringRotation(up, tire.m_steeringAngle);
	btMatrix3x3 steeringRotationMatrix(steeringRotation);

	btQuaternion tireRotation(right, -tire.m_currentRotation);
	btMatrix3x3 tireRotationMatrix(tireRotation);

	btMatrix3x3 tirePrincipalAxes;
	tirePrincipalAxes[0][0] = -right[0];
	tirePrincipalAxes[1][0] = -right[1];
	tirePrincipalAxes[2][0] = -right[2];

	tirePrincipalAxes[0][1] = up[0];
	tirePrincipalAxes[1][1] = up[1];
	tirePrincipalAxes[2][1] = up[2];

	tirePrincipalAxes[0][2] = fwd[0];
	tirePrincipalAxes[1][2] = fwd[1];
	tirePrincipalAxes[2][2] = fwd[2];

	tire.m_worldTransform.setBasis(steeringRotationMatrix * tireRotationMatrix * tirePrincipalAxes);
	tire.m_worldTransform.setOrigin(tire.m_ChassisConnectionPosition + tire.m_suspensionDir * tire.m_currentSuspensionLength);
}

void Vehicle::debugDraw(btIDebugDraw* debugDrawer)
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

	start = tire.m_ChassisConnectionPosition;
	btScalar totalLength = tire.m_radius + tire.m_suspensionLength;
	stop = start + tire.m_suspensionDir * totalLength;

	RaycastHit hitInfo;
	bool isHit = m_raycast->castRay(start, stop, hitInfo);

	if(isHit)
	{
		tire.m_isContactingGround = true;
		tire.m_groundContactPosition = hitInfo.m_positionWorld;
		tire.m_groundNormal = hitInfo.m_normalWorld;
		tire.m_penetrationDepth = (1 - hitInfo.m_closestHitFraction) * totalLength;
		tire.m_currentSuspensionLength = tire.m_suspensionLength - tire.m_penetrationDepth;

		///get relative position of tire contact to chassis CoM in world space
		btVector3 tireContactToCoM = m_chassisRigidBody->getCenterOfMassPosition() - tire.m_groundContactPosition;
		///relative velocity of the tire to the ground in world space
		btVector3 contactVelocity = m_chassisRigidBody->getVelocityInLocalPoint(tireContactToCoM);
		btScalar normalVelocity = contactVelocity.dot(tire.m_groundNormal);

		return  true;
	}

	tire.m_isContactingGround = false;
	tire.m_currentSuspensionLength = tire.m_suspensionLength;

	return false;
}



