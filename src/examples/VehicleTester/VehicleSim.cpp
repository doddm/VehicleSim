#include "btBulletDynamicsCommon.h"
#include "VehicleSim.h"
#include "../../graphics/CommonRenderInterface.h"

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
float suspensionDamping = 2.3f;
float suspensionCompression = 4.4f;
float rollInfluence = 0.1f;  //1.0f;

/// unit vector of suspension travel. It points from the axle to the contact patch
btVector3 tireSuspensionDirLocal(0, -1, 0);
/// unit vector indicating the direction of the axle
btVector3 tireAxleDirLocal(-1, 0, 0);

/// whether to render the wheels as boxes instead of cylinders
bool renderWheelsAsBoxes = false;

VehicleSim::VehicleSim(struct GUIHelperInterface* helper) :m_guiHelper(helper)
{
	// set y-up in GUI
	helper->setUpAxis(1);
}

void VehicleSim::initPhysics()
{
	/// set y-up coordinate system for GUI to match physics
	m_guiHelper->setUpAxis(1);

	/// collision configuration contains default setup for memory, collision setup.
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	/// use the default collision dispatcher.
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	/// btDbvtBroadphase is a good general purpose broadphase.
	m_overlappingPairCache = new btDbvtBroadphase();

	/// the default constraint solver (single threaded).
	m_constraintSolver = new btSequentialImpulseConstraintSolver;

	/// the high level interface for managing physics objects and constraints
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver,m_collisionConfiguration);

	m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

	initGroundTerrain();

	/// Create vehicle

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
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, bodyHeight, 0));

	compound->addChildShape(localTrans, chassisShape);

	btTransform tr;
	tr.setOrigin(btVector3(0, 0, 0));

	// vehicle mass [kg]
	float vehicleMass = 700.0f;
	m_vehicleChassis = localCreateRigidBody(vehicleMass, tr, compound);  //chassisShape);

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

	/// temporary position and rotation. tires will be re-positioned after they are added to the vehicle
	const float position[4] = { 0, 10, 10, 0 };
	const float quaternion[4] = { 0, 0, 0, 1 };
	/// dark gray color
	const float color[4] = { 0.2, 0.2, 0.2, 1 };
	const float scaling[4] = { 1, 1, 1, 1 };

	for (int i = 0; i < 4; i++)
	{
		m_tireRenderInstances[i] = m_guiHelper->registerGraphicsInstance(wheelGraphicsIndex, position, quaternion, color, scaling);
	}

	/// create vehicle
	{
		m_vehicle = new Vehicle(m_vehicleChassis);
//		m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
//		m_vehicle = new btRaycastVehicle(m_tuning, m_carChassis, m_vehicleRayCaster);
//
//		///never deactivate the vehicle
//		m_carChassis->setActivationState(DISABLE_DEACTIVATION);
//
		///Add the vehicle implementation of the Action Interface to the physics pipeline
		m_dynamicsWorld->addAction(m_vehicle);
//
//		bool isFrontWheel = true;
//
//		//choose coordinate system
//		m_vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);
//
		const float halfBodyLength = bodyLength * 0.5f;

		btVector3 connectionPointCS0(0.5f * wheelBaseFront, tireHeight, halfBodyLength - tireRadius);

		m_vehicle->AddTire(connectionPointCS0, tireAxleDirLocal, tireSuspensionDirLocal, wheelFriction, tireWidth, tireRadius);

//
//		// left front
//		btVector3 connectionPointCS0(0.5f * wheelBaseFront, tireHeight, halfBodyLength - tireRadius);
//
//		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, tireRadius,
//				m_tuning, isFrontWheel);
//
//		// right front
//		connectionPointCS0 = btVector3(-0.5f * wheelBaseFront, tireHeight, halfBodyLength - tireRadius);
//
//		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, tireRadius,
//				m_tuning, isFrontWheel);
//
//		// right rear
//		connectionPointCS0 = btVector3(-0.5f * wheelBaseRear, tireHeight,
//				-halfBodyLength + tireRadius);
//		isFrontWheel = false;
//		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, tireRadius,
//				m_tuning, isFrontWheel);
//
//		// left rear
//		connectionPointCS0 = btVector3(0.5f * wheelBaseRear, tireHeight, -halfBodyLength + tireRadius);
//		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, tireRadius,
//				m_tuning, isFrontWheel);
	}
	m_guiHelper->createCollisionShapeGraphicsObject(m_vehicleChassis->getCollisionShape());
	m_guiHelper->createCollisionObjectGraphicsObject(m_vehicleChassis, chassisColor);
}

void VehicleSim::initGroundTerrain()
{
	btBoxShape* ground = new btBoxShape(btVector3(0.5f * terrainExtent, 0.5f * terrainThickness, 0.5f * terrainExtent));
	m_collisionShapes.push_back(ground);

	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, -3, 0));

	btRigidBody* groundRigidBody = localCreateRigidBody(0, tr, ground);

	m_guiHelper->createCollisionShapeGraphicsObject(ground);

	m_guiHelper->createCollisionObjectGraphicsObject(groundRigidBody, terrainColor);
}

void VehicleSim::exitPhysics()
{}

void VehicleSim::stepSimulation(float deltaTime)
{
	if(m_dynamicsWorld)
	{
		int maxSimSubSteps = 2;
		m_dynamicsWorld->stepSimulation(deltaTime, maxSimSubSteps);
	}
}

void VehicleSim::renderScene()
{
	m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
	m_guiHelper->render(m_dynamicsWorld);

	for (int i = 0; i < m_vehicle->GetNumTires(); i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateTireTransform(i);

		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		if (renderer)
		{
			btTransform tr = m_vehicle->GetTire(i).m_worldTransform;
			btVector3 pos = tr.getOrigin();
			btQuaternion orn = tr.getRotation();
			renderer->writeSingleInstanceTransformToCPU(pos, orn, m_tireRenderInstances[i]);
		}
	}

	m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
	m_guiHelper->render(m_dynamicsWorld);

	ATTRIBUTE_ALIGNED16(btScalar)
			m[16];
	int i;

//	btVector3 worldBoundsMin, worldBoundsMax;
//	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin, worldBoundsMax);
//
//	for (i = 0; i < m_vehicle->getNumWheels(); i++)
//	{
//		//synchronize the wheels with the (interpolated) chassis worldtransform
//		m_vehicle->updateWheelTransform(i, true);
//		//draw wheels (cylinders)
//		m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
//	}
}

void VehicleSim::physicsDebugDraw(int debugFlags)
{}

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
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
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
