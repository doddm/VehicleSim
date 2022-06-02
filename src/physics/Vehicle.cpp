#include "Vehicle.h"
#include "../util/DebugUtil.h"

Vehicle::Vehicle(btRigidBody* pBody, Raycast* pRaycast)
{
	m_chassisRigidBody = pBody;
	m_raycast = pRaycast;
}

void Vehicle::update(btScalar step)
{
	///do raycast for all wheels and update wheel data with raycast result
	for (int i = 0; i < m_numWheels; i++)
	{
		updateTireWorldPositionRotation(m_tires[i]);
		updateTireWorldTransform(i);

		castRay(m_tires[i]);
	}

	updateSuspension(step);

	if (m_isTireFrictionActive)
	{
		updateTireFriction(step);
	}

	for (int i = 0; i < m_numWheels; i++)
	{
		Tire& tire = m_tires[i];
		btVector3 relpos = tire.m_chassisConnectionPosWorld - getRigidBody()->getCenterOfMassPosition();
		btVector3 vel = getRigidBody()->getVelocityInLocalPoint(relpos);

		if (tire.m_isContactingGround)
		{
			const btTransform& chassisWorldTransform = getChassisWorldTransform();

			btVector3 fwd(
				chassisWorldTransform.getBasis()[0][2],
				chassisWorldTransform.getBasis()[1][2],
				chassisWorldTransform.getBasis()[2][2]);

			btScalar proj = fwd.dot(tire.m_groundNormal);
			fwd -= tire.m_groundNormal * proj;

			btScalar proj2 = fwd.dot(vel);

			tire.m_deltaRotation = (proj2 * step) / (tire.m_radius);
			tire.m_currentRotation += tire.m_deltaRotation;
		}
		else
		{
			tire.m_currentRotation += tire.m_deltaRotation;
		}

		tire.m_deltaRotation *= btScalar(0.99);  //damping of rotation when not in contact
	}
}

// TODO Fix friction modeling
void Vehicle::updateTireFriction(btScalar step)
{
	for (int i = 0; i < m_numWheels; i++)
	{
		if (!m_tires[i].m_isContactingGround)
		{
			break;
		}

		btVector3 groundContactPosition = m_tires[i].m_groundContactPosition;
		btVector3 relativeVelocity = m_chassisRigidBody->getVelocityInLocalPoint(groundContactPosition);
		m_tires[i].m_groundContactVelWorld = relativeVelocity;
		m_tires[i].m_frictionDirWorld = -relativeVelocity.normalize();

		const btVector3& groundNormal = m_tires[i].m_groundNormal;

		const btMatrix3x3 tireBasisWorld = m_tires[i].m_worldTransform.getBasis();

		btVector3 tireAxleDirection = btVector3(
			tireBasisWorld[0][0],
			tireBasisWorld[1][0],
			tireBasisWorld[2][0]);

		m_tires[i].m_groundContactFwdWorld = tireAxleDirection.cross(groundNormal);

		// get ground contact longitudinal component of velocity. Project relative velocity on ground forward dir
		btScalar longComp = -relativeVelocity.dot(m_tires[i].m_groundContactFwdWorld);
		m_tires[i].m_lonFrictionDirWorld = longComp * m_tires[i].m_groundContactFwdWorld;

		// get ground contact lateral component of velocity. Project relative velocity on ground later dir (axle dir)
		btScalar latComp = -relativeVelocity.dot(tireAxleDirection);
		m_tires[i].m_latFrictionDirWorld = latComp * tireAxleDirection;

		btScalar axelNormalComp = groundNormal.dot(tireAxleDirection);

		btVector3 tireAxleDirectionPerpGround = tireAxleDirection - axelNormalComp * groundNormal;
		tireAxleDirectionPerpGround.normalize();

		btVector3 relativePosition = m_tires[i].m_groundContactPosition - m_chassisRigidBody->getCenterOfMassPosition();

		float impulseMulti = abs(m_tires[i].m_suspensionForce);

		btVector3 forwardImpulse = m_tires[i].m_groundContactFwdWorld * (m_tires[i].m_engineTorque + m_tires[i].m_brakeTorque);
		btVector3 slipImpulse = m_tires[i].m_latFrictionDirWorld * impulseMulti;

		m_chassisRigidBody->applyForce(forwardImpulse, relativePosition);
		m_chassisRigidBody->applyForce(slipImpulse, relativePosition);
	}
}
void Vehicle::updateSuspension(btScalar step)
{
	btScalar mass = m_chassisRigidBody->getMass();
	for (int i = 0; i < m_numWheels; i++)
	{
		Tire& tire = m_tires[i];
		if (!tire.m_isContactingGround)
		{
			tire.m_suspensionForce = btScalar(0.0);
			break;
		}
		btScalar displacement = tire.m_currentSuspensionLength - tire.m_suspensionLength;
		btScalar springForceMag = displacement * m_suspensionStiffness;
		btVector3 springImpulse = springForceMag * tire.m_suspensionDirWorld;
		btVector3 relativePosition = tire.m_groundContactPosition - m_chassisRigidBody->getCenterOfMassPosition();

		btScalar damperForceMag = tire.m_velocityLocal * m_suspensionDamping;
		btVector3 damperImpulse = damperForceMag * tire.m_suspensionDirWorld;

		// prevent negative force
		tire.m_suspensionForce = btMax(btScalar(0.0), -mass * (springForceMag + damperForceMag));
		btVector3 suspensionImpulse = step * mass * (springImpulse + damperImpulse);

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

Tire& Vehicle::addTire(const btVector3& position,
	const btVector3& rotationAxis,
	const btVector3& suspensionDir,
	const btScalar suspensionLength,
	btScalar friction,
	btScalar width,
	btScalar radius,
	bool isSteerable)
{
	Tire tireToAdd(position, rotationAxis, suspensionDir, suspensionLength, radius, width, friction, isSteerable);
	m_tires.push_back(tireToAdd);
	updateTireWorldPositionRotation(tireToAdd);
	updateTireWorldTransform(getNumTires() - 1);
	return tireToAdd;
}

const Tire& Vehicle::getTire(int tireIndex) const
{
	return m_tires[tireIndex];
}

void Vehicle::updateTireWorldPositionRotation(Tire& tire)
{
	btTransform chassisTransform = getChassisWorldTransform();
	tire.m_chassisConnectionPosWorld = chassisTransform(tire.m_chassisConnectionPosLocal);
	tire.m_rotationAxisWorld = chassisTransform.getBasis() * tire.m_rotationAxisLocal;
	tire.m_suspensionDirWorld = chassisTransform.getBasis() * tire.m_suspensionDirLocal;
	return;
}

void Vehicle::updateTireWorldTransform(int tireIndex)
{
	Tire& tire = m_tires[tireIndex];
	updateTireWorldPositionRotation(tire);
	btVector3 up = -tire.m_suspensionDirWorld;
	const btVector3& right = tire.m_rotationAxisWorld;
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
	tire.m_worldTransform.setOrigin(tire.m_chassisConnectionPosWorld + tire.m_suspensionDirWorld * tire.m_currentSuspensionLength);
}

void Vehicle::debugDraw(btIDebugDraw* debugDrawer)
{
	for (int i = 0; i < 4; i++)
	{
		btVector3 tireDirsColor;
		if (m_tires[i].m_isContactingGround)
		{
			tireDirsColor.setValue(0, 0, 1);
		}
		else
		{
			tireDirsColor.setValue(1, 0, 0);
		}

		btVector3 wheelPosWS = m_tires[i].m_worldTransform.getOrigin();

		btVector3 axle = btVector3(
			m_tires[i].m_worldTransform.getBasis()[0][0],
			m_tires[i].m_worldTransform.getBasis()[1][0],
			m_tires[i].m_worldTransform.getBasis()[2][0]);

		// draw wheel axle as line
		debugDrawer->drawLine(wheelPosWS, wheelPosWS + axle, tireDirsColor);
		// draw line from wheel to ground contact position
		debugDrawer->drawLine(wheelPosWS, m_tires[i].m_groundContactPosition, tireDirsColor);

		btVector3 tireFrictionColor{1, 1, 0};
		if (m_tires[i].m_isContactingGround)
		{
			// draw friction directions
			debugDrawer->drawLine(m_tires[i].m_groundContactPosition, m_tires[i].m_groundContactPosition + m_tires[i].m_lonFrictionDirWorld, tireFrictionColor);
			debugDrawer->drawLine(m_tires[i].m_groundContactPosition, m_tires[i].m_groundContactPosition + m_tires[i].m_latFrictionDirWorld, tireFrictionColor);

			// draw engine and steering force directions
			tireDirsColor.setValue(1, 0, 0);
			debugDrawer->drawLine(m_tires[i].m_groundContactPosition, m_tires[i].m_groundContactPosition + m_tires[i].m_groundContactFwdWorld, tireDirsColor);
			debugDrawer->drawLine(m_tires[i].m_groundContactPosition, m_tires[i].m_groundContactPosition + m_tires[i].m_rotationAxisWorld, tireDirsColor);
		}
		debugDrawer->drawSphere(m_tires[i].m_groundContactPosition, 0.1, tireDirsColor);
	}
}

void Vehicle::setBrake(btScalar brakeForce)
{
	if (m_isTireFrictionActive)
	{
		for (int i = 0; i < m_numWheels; i++)
		{
			m_tires[i].m_brakeTorque = -brakeForce;
		}
	}
	else
	{
		btMatrix3x3 vehicleBasis = m_chassisRigidBody->getWorldTransform().getBasis();
		btVector3 backwardWorld = -btVector3(vehicleBasis[0][2], vehicleBasis[1][2], vehicleBasis[2][2]);
		m_chassisRigidBody->applyCentralForce(backwardWorld * brakeForce);
	}
}

void Vehicle::setSteering(btScalar angle)
{
	if (m_isTireFrictionActive)
	{
		for (int i = 0; i < m_numWheels; i++)
		{
			if (m_tires[i].isSteerable)
			{
				m_tires[i].m_steeringAngle = angle;
			}
		}
	}
	else
	{
		btMatrix3x3 vehicleBasis = m_chassisRigidBody->getWorldTransform().getBasis();
		btVector3 upWorld = -btVector3(vehicleBasis[0][1], vehicleBasis[1][1], vehicleBasis[2][1]);
		m_chassisRigidBody->applyTorque(-1000 * upWorld * angle);
	}
}

void Vehicle::setAccelerator(btScalar engineForce)
{
	if (m_isTireFrictionActive)
	{
		for (int i = 0; i < m_numWheels; i++)
		{
			if (!m_tires[i].isSteerable)
			{
				m_tires[i].m_engineTorque = engineForce;
			}
		}
	}
	else
	{
		btMatrix3x3 vehicleBasis = m_chassisRigidBody->getWorldTransform().getBasis();
		btVector3 forwardWorld = btVector3(vehicleBasis[0][2], vehicleBasis[1][2], vehicleBasis[2][2]);
		m_chassisRigidBody->applyCentralForce(forwardWorld * engineForce);
	}
}

Vehicle::~Vehicle()
{

}

bool Vehicle::castRay(Tire& tire)
{
	btVector3 start;
	btVector3 stop;

	start = tire.m_chassisConnectionPosWorld;
	btScalar totalLength = tire.m_radius + tire.m_suspensionLength;
	stop = start + tire.m_suspensionDirWorld * totalLength;

	RaycastHit hitInfo;
	bool isHit = m_raycast->castRay(start, stop, hitInfo);

	if (isHit)
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
		tire.m_velocityLocal = normalVelocity;


		// TODO figure out why this bit from btvehicle works but the above is unstable
		btScalar denominator = tire.m_groundNormal.dot(tire.m_suspensionDirWorld);

		btVector3 chassis_velocity_at_contactPoint;
		btVector3 relpos = tire.m_groundContactPosition - getRigidBody()->getCenterOfMassPosition();

		chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

		btScalar projVel = tire.m_groundNormal.dot(chassis_velocity_at_contactPoint);

		btScalar inv = btScalar(-1.) / denominator;
		tire.m_velocityLocal = projVel * inv;

		return true;
	}

	tire.m_isContactingGround = false;
	tire.m_currentSuspensionLength = tire.m_suspensionLength;
	btVector3& tireOrigin = tire.m_chassisConnectionPosWorld;
	// TODO double check this math
	tire.m_groundContactPosition = tireOrigin + tire.m_suspensionLength * tire.m_suspensionDirWorld;

	return false;
}
void Vehicle::setSuspensionStiffness(btScalar stiffness)
{
	m_suspensionStiffness = stiffness;
}

void Vehicle::setSuspensionDamping(btScalar damping)
{
	m_suspensionDamping = damping;
}
void Vehicle::setTireFrictionActive(bool isActive)
{
	m_isTireFrictionActive = isActive;
}
