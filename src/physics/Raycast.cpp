#include <iostream>
#include "Raycast.h"

Raycast::Raycast(btDynamicsWorld *dynamicsWorld)
    : m_dynamicsWorld(dynamicsWorld) {}

// TODO change "to" parameter to be a "direction" and "length"
bool Raycast::castRay(btVector3 from, btVector3 to, RaycastHit &hitInfo) const
{
  btCollisionWorld::ClosestRayResultCallback rayCallback(from, to);

  m_dynamicsWorld->rayTest(from, to, rayCallback);

  if (rayCallback.hasHit())
  {
    const btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
    if (body && body->hasContactResponse())
    {
      hitInfo.m_positionWorld = rayCallback.m_hitPointWorld;
      hitInfo.m_normalWorld = rayCallback.m_hitNormalWorld;
      hitInfo.m_normalWorld.normalize();
      hitInfo.m_closestHitFraction = rayCallback.m_closestHitFraction;
    }
    return true;
  }
  return false;
}
void RaycastHit::debugPrint()
{
  std::cout << "ray hit position x:" << m_positionWorld.x() << " y:" << m_positionWorld.y() << " z:" << m_positionWorld.z() << std::endl;
  std::cout << "ray hit normal x:" << m_normalWorld.x() << " y:" << m_normalWorld.y() << " z:" << m_normalWorld.z() << std::endl;
  std::cout << "ray hit fraction :" << m_closestHitFraction << std::endl;
}
