#include "btBulletDynamicsCommon.h"
#include "VehicleSim.h"

void VehicleSim::Initialize()
{
	///set y-up coordinate system for GUI to match physics
	m_guiHelper->setUpAxis(1);

	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	m_overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	m_constraintSolver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver, m_collisionConfiguration);

	m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
}
VehicleSim::VehicleSim(struct GUIHelperInterface* helper) : m_guiHelper(helper)
{
	// set y-up in GUI
	helper->setUpAxis(1);
}

