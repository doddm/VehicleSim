//
// Created by mchld on 5/25/2022.
//

#ifndef BULLETGAME_VEHICLESIM_H
#define BULLETGAME_VEHICLESIM_H

#include "examples/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"

class VehicleSim : public CommonExampleInterface
{
public:
	GUIHelperInterface* m_guiHelper;

	class btDiscreteDynamicsWorld* m_dynamicsWorld;

	class btBroadphaseInterface* m_overlappingPairCache;

	class btCollisionDispatcher* m_dispatcher;

	class btConstraintSolver* m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	VehicleSim::VehicleSim(struct GUIHelperInterface* helper);

	void Initialize();
	void Update();

};

#endif //BULLETGAME_VEHICLESIM_H
