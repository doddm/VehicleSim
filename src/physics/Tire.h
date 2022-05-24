//
// Created by mchld on 5/23/2022.
//

#ifndef BULLETGAME_SRC_PHYSICS_TIRE_H_
#define BULLETGAME_SRC_PHYSICS_TIRE_H_

#include "LinearMath/btScalar.h"
class Tire
{
 public:
	btScalar m_radius;
	btScalar m_width;
	btScalar m_friction;
	btScalar m_suspensionStiffness;
	btScalar m_suspensionDamping;
	btScalar m_engineTorque;
	btScalar m_brakeTorque;

};

#endif //BULLETGAME_SRC_PHYSICS_TIRE_H_
