//
// Created by mchld on 5/25/2022.
//

#ifndef BULLETGAME_VEHICLESIM_H
#define BULLETGAME_VEHICLESIM_H

#include "examples/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "physics/Vehicle.h"

class CommonExampleInterface* VehicleSimCreateFunc(struct CommonExampleOptions& options);

class VehicleSim : public CommonExampleInterface
{
public:
	const btVector4 terrainColor{ 112. / 256., 129. / 256., 87. / 256., 1 };
	const btVector4 chassisColor{ 72. / 256., 133. / 256., 237. / 256., 1 };

	GUIHelperInterface* m_guiHelper;

	btDiscreteDynamicsWorld* m_dynamicsWorld;

	btBroadphaseInterface* m_overlappingPairCache;

	btCollisionDispatcher* m_dispatcher;

	btConstraintSolver* m_constraintSolver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	btRigidBody* m_vehicleChassis;

	Vehicle* m_vehicle;

	btCollisionShape* m_tireShape;

	int m_tireRenderInstances[4];

	explicit VehicleSim(struct GUIHelperInterface* helper);

	void initPhysics() override;
	void exitPhysics() override;
	void stepSimulation(float deltaTime) override;
	void renderScene() override;
	void physicsDebugDraw(int debugFlags) override;
	void resetCamera() override;
	bool mouseMoveCallback(float x, float y) override;
	bool mouseButtonCallback(int button, int state, float x, float y) override;
	bool keyboardCallback(int key, int state) override;

	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

private:
	void initGroundTerrain();
};

#endif //BULLETGAME_VEHICLESIM_H