//
// Created by Michael Dodd 2022.
//

#ifndef BULLETGAME_RAYCAST_H
#define BULLETGAME_RAYCAST_H

#include "Raycast.h"
#include "btBulletDynamicsCommon.h"

struct RaycastHit {
public:
  btVector3 m_positionWorld;
  btVector3 m_normalWorld;
  btScalar m_closestHitFraction;

  void debugPrint();
};

class Raycast {
public:
  Raycast(btDynamicsWorld *dynamicsWorld);
  bool castRay(btVector3 from, btVector3 to, RaycastHit &hitInfo) const;

private:
  btDynamicsWorld *m_dynamicsWorld;
};

#endif // BULLETGAME_RAYCAST_H
