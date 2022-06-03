#include "Aerodynamics.h"
#include "btBulletDynamicsCommon.h"

void Aerodynamics::Update()
{
	/// calculate and apply simple aerodynamic drag force to chassis
	btVector3 velocity = m_rigidBody->getLinearVelocity();
	btScalar velocityMag = velocity.length();
	btScalar dragScalar = 0.5 * m_surfaceArea * velocityMag * velocityMag * k_coefficientOfDrag;

	m_rigidBody->applyCentralForce(-dragScalar * velocity.normalize());
}

Aerodynamics::Aerodynamics(btRigidBody* pBody)
{
	m_rigidBody = pBody;
	btCollisionShape* collisionShape = m_rigidBody->getCollisionShape();

	/// approximate the box as a sphere for getting frontal area for drag calculation (justifiable if box is cube-like)
	btVector3 center;
	btScalar radius;
	collisionShape->getBoundingSphere(center, radius);

	m_surfaceArea = SIMD_PI * radius * radius;
}
