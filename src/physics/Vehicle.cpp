#include "Vehicle.h"
#include "util/DebugUtil.h"

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

	updateTireFriction(step);
}

void Vehicle::updateTireFriction(btScalar step)
{
	for(int i = 0; i < numWheels; i++)
	{
		if(!m_tires[i].m_isContactingGround)
		{
			break;
		}

		btVector3 groundContactPosition = m_tires[i].m_groundContactPosition;
		btVector3 slipVelocity = m_chassisRigidBody->getVelocityInLocalPoint(groundContactPosition);
		btVector3 tireAxleDirection = m_tires[i].m_rotationAxis;

		// todo extend this for arbitrary terrain normals
		btVector3 groundNormal(0, 1, 0);
		btVector3 tireForward = -tireAxleDirection.cross(groundNormal);

		btMatrix3x3 tireBasisWorld = m_tires[i].m_worldTransform.getBasis();
		tireForward = btVector3(
			tireBasisWorld[0][2],
			tireBasisWorld[1][2],
			tireBasisWorld[2][2]);

		btMatrix3x3 tireAxelWorld = m_tires[i].m_worldTransform.getBasis();
		tireAxleDirection = btVector3(
			tireBasisWorld[0][0],
			tireBasisWorld[1][0],
			tireBasisWorld[2][0]);

		btVector3 relativePosition = m_tires[i].m_groundContactPosition - m_chassisRigidBody->getCenterOfMassPosition();

		float impulseMulti = 10;

		btScalar forwardImpulseMag = tireForward.dot(-slipVelocity);
		forwardImpulseMag *= impulseMulti;
		forwardImpulseMag = 0;
		btVector3 forwardImpulse = forwardImpulseMag * tireForward.normalize();

		btScalar slipImpulseMag = tireAxleDirection.dot(-slipVelocity);
		slipImpulseMag *= impulseMulti;
		btVector3 slipImpulse = slipImpulseMag * tireAxleDirection.normalize();

		m_chassisRigidBody->applyImpulse(forwardImpulse, relativePosition);
		m_chassisRigidBody->applyImpulse(slipImpulse, relativePosition);

//		DebugUtil::printVector3(tireForward, "forward vector ");
//		DebugUtil::printVector3(slipVelocity, "slip velocity ");



		if(m_tires[i].isSteerable)
		{
//			DebugUtil::printVector3(tireForwardWS, "Tire forward WS ");
			DebugUtil::printVector3(tireForward, "Tire forward Local? ");
		}
	}
}
void Vehicle::updateSuspension(btScalar step)
{
	btScalar mass = m_chassisRigidBody->getMass()/numWheels;
	for(int i = 0; i < numWheels; i++)
	{
		Tire& tire = m_tires[i];
		btScalar displacement = tire.m_currentSuspensionLength - tire.m_suspensionLength;
		btScalar springForceMag = displacement * m_suspensionStiffness;
		btVector3 springImpulse = springForceMag * step * tire.m_suspensionDir;
		btVector3 relativePosition = tire.m_groundContactPosition - m_chassisRigidBody->getCenterOfMassPosition();


		btScalar damperForceMag = tire.m_localVelocity * m_suspensionDamping;
		btVector3 damperImpulse = damperForceMag * step * tire.m_suspensionDir;

		btVector3 suspensionImpulse = mass*(springImpulse + damperImpulse);

		m_chassisRigidBody->applyImpulse(suspensionImpulse, relativePosition);
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

Tire& Vehicle::addTire(const btVector3& position, const btVector3& rotationAxis, const btVector3& suspensionDir, const btScalar suspensionLength, btScalar friction, btScalar width, btScalar radius, bool isSteerable)
{
	Tire tireToAdd(position, rotationAxis, suspensionDir, suspensionLength, radius, width, friction, isSteerable);
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
void Vehicle::setBrake(btScalar brakeForce)
{
	m_chassisRigidBody->applyCentralForce(brakeForce * btVector3(0,0,-1));
}

void Vehicle::setSteering(btScalar angle)
{
	for(int i = 0; i < numWheels; i++)
	{
		if(m_tires[i].isSteerable)
		{
			m_tires[i].m_steeringAngle = angle;
		}
	}
}

void Vehicle::setAccelerator(btScalar engineForce)
{
	m_chassisRigidBody->applyCentralForce(engineForce * btVector3(0,0,1));
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
		tire.m_localVelocity = normalVelocity;


		// TODO figure out why this works but the above is unstable
		btScalar denominator = tire.m_groundNormal.dot(tire.m_suspensionDir);

		btVector3 chassis_velocity_at_contactPoint;
		btVector3 relpos = tire.m_groundContactPosition - getRigidBody()->getCenterOfMassPosition();

		chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

		btScalar projVel = tire.m_groundNormal.dot(chassis_velocity_at_contactPoint);

//		if (denominator >= btScalar(-0.1))
//		{
//			wheel.m_suspensionRelativeVelocity = btScalar(0.0);
//			wheel.m_clippedInvContactDotSuspension = btScalar(1.0) / btScalar(0.1);
//		}
//		else
//		{
			btScalar inv = btScalar(-1.) / denominator;
			tire.m_localVelocity = projVel * inv;
//			wheel.m_clippedInvContactDotSuspension = inv;
//		}

		return  true;
	}

	tire.m_isContactingGround = false;
	tire.m_currentSuspensionLength = tire.m_suspensionLength;

	return false;
}
void Vehicle::setSuspensionStiffness(btScalar stiffness)
{
	m_suspensionStiffness= stiffness;
}
btScalar Vehicle::getSuspensionStiffness()
{
	return m_suspensionStiffness;
}
void Vehicle::setSuspensionDamping(btScalar damping)
{
	m_suspensionDamping = damping;
}
btScalar Vehicle::getSuspensionDamping()
{
	return m_suspensionDamping;
}
