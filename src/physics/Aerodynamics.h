#ifndef BULLETGAME_SRC_PHYSICS_AERODYNAMICS_H_
#define BULLETGAME_SRC_PHYSICS_AERODYNAMICS_H_

#include "BulletDynamics/Dynamics/btRigidBody.h"
class Aerodynamics
{
 public:
	const btScalar k_coefficientOfDrag{ 0.5};

	explicit Aerodynamics(btRigidBody* pBody);
	void Update();

 private:
	btRigidBody* m_rigidBody;
	btScalar m_surfaceArea;
};

#endif //BULLETGAME_SRC_PHYSICS_AERODYNAMICS_H_
