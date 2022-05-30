#include "VehicleSim.h"
#include "../../graphics/CommonRenderInterface.h"
#include "btBulletDynamicsCommon.h"
#include <iostream>

/// terrain dimensions [m]
float terrainExtent = 100;
float terrainThickness = 10;

/// vehicle body dimensions [m]
float bodyWidth = 1.9f;
float bodyHeight = 1.8f;
float bodyLength = 3.5f;

/// tire dimensions [m]
float tireHeight = 0.8f;
float tireRadius = 0.35f;
float tireWidth = 0.3f;

float gVehicleSteering = 0.f;
float steeringIncrement = 0.04f;
float steeringClamp = 0.3f;
float wheelBaseFront = 2.1f;
float wheelBaseRear = wheelBaseFront;
float wheelFriction = 100;
float suspensionStiffness = 20.f;
//float suspensionDamping = 2.3f;
float suspensionDamping = 0.0f;
float suspensionCompression = 4.4f;
float suspensionLength = 0.6;
float rollInfluence = 0.1f; // 1.0f;

/// unit vector of suspension travel.
btVector3 tireSuspensionDirLocal(0, -1, 0);
/// unit vector indicating the direction of the axle
btVector3 tireAxleDirLocal(-1, 0, 0);

/// whether to render the wheels as boxes instead of cylinders
bool renderWheelsAsBoxes = false;

VehicleSim::VehicleSim(struct GUIHelperInterface* helper) : m_guiHelper(helper)
{
	// set y-up in GUI
	helper->setUpAxis(1);
}

void VehicleSim::initPhysics()
{
	/// set y-up coordinate system for GUI to match physics
	m_guiHelper->setUpAxis(1);

	/// collision configuration contains default setup for memory, collision
	/// setup.
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	/// use the default collision dispatcher.
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	/// btDbvtBroadphase is a good general purpose broadphase.
	m_overlappingPairCache = new btDbvtBroadphase();

	/// the default constraint solver (single threaded).
	m_constraintSolver = new btSequentialImpulseConstraintSolver;

	/// the high level interface for managing physics objects and constraints
	m_dynamicsWorld =
		new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver, m_collisionConfiguration);

	m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.00001;

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

//		m_dynamicsWorld->setGravity(btVector3(0, -1.0, 0));

	initGroundTerrain();

	/// Chassis coordinate system
	/// +x - left
	/// +y - up
	/// +z - forward
	btCollisionShape* chassisShape = new btBoxShape(btVector3(0.5f * bodyWidth, 0.5f * bodyHeight, 0.5f * bodyLength));
	m_collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();

	// localTrans effectively shifts the center of mass with respect to the
	// chassis
	localTrans.setOrigin(btVector3(0, bodyHeight, 0));

	compound->addChildShape(localTrans, chassisShape);

	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, 0, 0));

	// vehicle mass [kg]
	float vehicleMass = 700.0f;
	m_vehicleChassis = localCreateRigidBody(vehicleMass, tr, compound); // chassisShape);

	if (renderWheelsAsBoxes)
	{
		m_tireShape = new btBoxShape(btVector3(0.5f * tireWidth, tireRadius, tireRadius));
	}
	else
	{
		m_tireShape = new btCylinderShapeX(btVector3(0.5f * tireWidth, tireRadius, tireRadius));
	}

	m_guiHelper->createCollisionShapeGraphicsObject(m_tireShape);
	int wheelGraphicsIndex = m_tireShape->getUserIndex();

	/// temporary position and rotation. tires will be re-positioned after they
	/// are added to the vehicle
	const float position[4] = { 0, 10, 10, 0 };
	const float quaternion[4] = { 0, 0, 0, 1 };
	const float scaling[4] = { 1, 1, 1, 1 };

	for (int i = 0; i < 4; i++)
	{
		m_tireRenderInstances[i] =
			m_guiHelper->registerGraphicsInstance(wheelGraphicsIndex, position, quaternion, tireColor, scaling);
	}

	createVehicle();

	m_guiHelper->createCollisionShapeGraphicsObject(m_vehicleChassis->getCollisionShape());
	m_guiHelper->createCollisionObjectGraphicsObject(m_vehicleChassis, chassisColor);
}
void VehicleSim::createVehicle()
{
	m_vehicleRaycast = new Raycast(m_dynamicsWorld);
	m_vehicle = new Vehicle(m_vehicleChassis, m_vehicleRaycast);

	/// never deactivate the vehicle
	m_vehicleChassis->setActivationState(DISABLE_DEACTIVATION);

	/// Add the vehicle implementation of the Action Interface to the physics
	/// pipeline
	m_dynamicsWorld->addAction(m_vehicle);

	addTiresToVehicle();
}
void VehicleSim::addTiresToVehicle()
{
	const float halfBodyLength = bodyLength * 0.5f;

	btVector3 leftFrontTirePositionLocal(0.5f * wheelBaseFront, tireHeight, halfBodyLength - tireRadius);
	m_vehicle->addTire(leftFrontTirePositionLocal,
		tireAxleDirLocal,
		tireSuspensionDirLocal,
		suspensionLength,
		wheelFriction,
		tireWidth,
		tireRadius,
		suspensionStiffness);

	btVector3 rightFrontTirePositionLocal(-0.5f * wheelBaseFront, tireHeight, halfBodyLength - tireRadius);
	m_vehicle->addTire(rightFrontTirePositionLocal,
		tireAxleDirLocal,
		tireSuspensionDirLocal,
		suspensionLength,
		wheelFriction,
		tireWidth,
		tireRadius,
		suspensionStiffness);

	btVector3 leftRearTirePositionLocal(0.5f * wheelBaseFront, tireHeight, tireRadius - halfBodyLength);
	m_vehicle->addTire(leftRearTirePositionLocal,
		tireAxleDirLocal,
		tireSuspensionDirLocal,
		suspensionLength,
		wheelFriction,
		tireWidth,
		tireRadius,
		suspensionStiffness);

	btVector3 rightRearTirePositionLocal(-0.5f * wheelBaseFront, tireHeight, tireRadius - halfBodyLength);
	m_vehicle->addTire(rightRearTirePositionLocal,
		tireAxleDirLocal,
		tireSuspensionDirLocal,
		suspensionLength,
		wheelFriction,
		tireWidth,
		tireRadius,
		suspensionStiffness);
}

void VehicleSim::initGroundTerrain()
{
	btBoxShape* ground = new btBoxShape(btVector3(0.5f * terrainExtent, 0.5f * terrainThickness, 0.5f * terrainExtent));
	m_collisionShapes.push_back(ground);

	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, -5, 0));

	btRigidBody* groundRigidBody = localCreateRigidBody(0, tr, ground);

	m_guiHelper->createCollisionShapeGraphicsObject(ground);

	m_guiHelper->createCollisionObjectGraphicsObject(groundRigidBody, terrainColor);
}

void VehicleSim::exitPhysics()
{
	std::cout << "EXITING PHYSICS" << std::endl;
	// remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			while (body->getNumConstraintRefs())
			{
				btTypedConstraint* constraint = body->getConstraintRef(0);
				m_dynamicsWorld->removeConstraint(constraint);
				delete constraint;
			}
			delete body->getMotionState();
			m_dynamicsWorld->removeRigidBody(body);
		}
		else
		{
			m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}

	// delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	// delete dynamics world
	delete m_dynamicsWorld;
	m_dynamicsWorld = nullptr;

	delete m_vehicle;
	m_vehicle = nullptr;

	delete m_vehicle;
	m_vehicle = nullptr;

	delete m_tireShape;
	m_tireShape = nullptr;

	// delete solver
	delete m_constraintSolver;
	m_constraintSolver = nullptr;

	// delete broadphase
	delete m_overlappingPairCache;
	m_overlappingPairCache = nullptr;

	// delete dispatcher
	delete m_dispatcher;
	m_dispatcher = nullptr;

	delete m_collisionConfiguration;
	m_collisionConfiguration = nullptr;
}

void VehicleSim::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
		int maxSimSubSteps = 2;
		m_dynamicsWorld->stepSimulation(deltaTime, maxSimSubSteps);
	}
}

void VehicleSim::renderScene()
{
	m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
	m_guiHelper->render(m_dynamicsWorld);

	for (int i = 0; i < m_vehicle->getNumTires(); i++)
	{
		// synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->setTireWorldTransform(i);

		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		if (renderer)
		{
			btTransform tr = m_vehicle->getTire(i).m_worldTransform;
			btVector3 pos = tr.getOrigin();
			//			std::cout << pos.length() << std::endl;
			btQuaternion orn = tr.getRotation();
			renderer->writeSingleInstanceTransformToCPU(pos, orn, m_tireRenderInstances[i]);
		}
	}

	m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
	m_guiHelper->render(m_dynamicsWorld);
}

void VehicleSim::physicsDebugDraw(int debugFlags)
{
}

bool VehicleSim::mouseMoveCallback(float x, float y)
{
	return true;
}

bool VehicleSim::mouseButtonCallback(int button, int state, float x, float y)
{
	return true;
}

bool VehicleSim::keyboardCallback(int key, int state)
{
	return true;
}

btRigidBody* VehicleSim::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	// rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
	{
		shape->calculateLocalInertia(mass, localInertia);
	}

	// MotionState is used to interpolate rigidbody positions between physics ticks
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

	btRigidBody* body = new btRigidBody(cInfo);

	m_dynamicsWorld->addRigidBody(body);
	return body;
}

void VehicleSim::resetCamera()
{
	float dist = 8;
	float pitch = -32;
	float yaw = -45;
	float targetPos[3] = { -0.33, -0.72, 4.5 };
	m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
}

CommonExampleInterface* VehicleSimCreateFunc(struct CommonExampleOptions& options)
{
	return new VehicleSim(options.m_guiHelper);
}
