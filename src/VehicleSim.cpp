// Adapted from Demos in Bullet Physics SDK https://bulletphysics.org

#include "VehicleSim.h"
#include "btBulletDynamicsCommon.h"
#include "graphics/CommonRenderInterface.h"
#include "physics/CommonRigidBodyBase.h"

#include <iostream>

float gravity = 9.81;

float terrainExtent = 100;
float terrainThickness = 10;

float bodyWidth = 1.9f;
float bodyHeight = 1.8f;
float bodyLength = 3.5f;

float tireHeight = 0.8f;
float tireRadius = 0.35f;
float tireWidth = 0.3f;

float currentSteeringAngle = 0.f;
float defaultSteeringAngle = 0.3f;
float defaultBrakingForce = 10.f;
float maxBrakingForce = 1000.f;
float currentBrakingForce = 0.f;
float maxEngineForce = 5000.f;
float currentEngineForce = 0.f;

float steeringIncrement = 0.04f;
float wheelBaseFront = 2.1f;
float wheelFriction = 100;
float suspensionStiffness = 20.f;
float suspensionDamping = 3.0f;
float suspensionLength = 0.9;

///(0 - plane, 1 - hills)
int terrainType = 1;

/// unit vector of suspension travel.
btVector3 tireSuspensionDirLocal(0, -1, 0);
/// unit vector indicating the direction of the tire axle
btVector3 tireAxleDirLocal(-1, 0, 0);

bool renderWheelsAsBoxes = true;

bool isTireFrictionActive = false;

VehicleSim::VehicleSim(struct GUIHelperInterface* helper) : m_guiHelper(helper)
{
	// set y-up in GUI
	helper->setUpAxis(1);
}

void VehicleSim::initPhysics()
{
	/// set y-up coordinate system for GUI to match physics
	m_guiHelper->setUpAxis(1);

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

	m_dynamicsWorld->setGravity(btVector3(0, -gravity, 0));

	initGroundTerrain(terrainType);

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

	// localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, bodyHeight, 0));

	compound->addChildShape(localTrans, chassisShape);

	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, 0, 0));

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

	/// Add the vehicle implementation of the Action Interface to the physics pipeline
	m_dynamicsWorld->addAction(m_vehicle);

	/// configure vehicle suspension
	m_vehicle->setSuspensionStiffness(suspensionStiffness);
	m_vehicle->setSuspensionDamping(suspensionDamping);

	m_vehicle->setTireFrictionActive(isTireFrictionActive);

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
		true);

	btVector3 rightFrontTirePositionLocal(-0.5f * wheelBaseFront, tireHeight, halfBodyLength - tireRadius);
	m_vehicle->addTire(rightFrontTirePositionLocal,
		tireAxleDirLocal,
		tireSuspensionDirLocal,
		suspensionLength,
		wheelFriction,
		tireWidth,
		tireRadius,
		true);

	btVector3 leftRearTirePositionLocal(0.5f * wheelBaseFront, tireHeight, tireRadius - halfBodyLength);
	m_vehicle->addTire(leftRearTirePositionLocal,
		tireAxleDirLocal,
		tireSuspensionDirLocal,
		suspensionLength,
		wheelFriction,
		tireWidth,
		tireRadius,
		false);

	btVector3 rightRearTirePositionLocal(-0.5f * wheelBaseFront, tireHeight, tireRadius - halfBodyLength);
	m_vehicle->addTire(rightRearTirePositionLocal,
		tireAxleDirLocal,
		tireSuspensionDirLocal,
		suspensionLength,
		wheelFriction,
		tireWidth,
		tireRadius,
		false);
}

void VehicleSim::exitPhysics()
{
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

	delete m_dynamicsWorld;
	m_dynamicsWorld = nullptr;

	delete m_vehicle;
	m_vehicle = nullptr;

	delete m_vehicle;
	m_vehicle = nullptr;

	delete m_tireShape;
	m_tireShape = nullptr;

	delete m_constraintSolver;
	m_constraintSolver = nullptr;

	delete m_overlappingPairCache;
	m_overlappingPairCache = nullptr;

	delete m_dispatcher;
	m_dispatcher = nullptr;

	delete m_collisionConfiguration;
	m_collisionConfiguration = nullptr;
}

void VehicleSim::stepSimulation(float deltaTime)
{
	m_vehicle->setSteering(currentSteeringAngle);
	m_vehicle->setAccelerator(currentEngineForce);
	m_vehicle->setBrake(currentBrakingForce);

	if (m_dynamicsWorld)
	{
		int maxSimSubSteps = 2;
		m_dynamicsWorld->stepSimulation(deltaTime, maxSimSubSteps);

		if(m_vehicleChassis->getCenterOfMassPosition().y() < -20)
		{
			resetVehicle(btVector3(0, 0, 0));
		}
	}
}

void VehicleSim::renderScene()
{
	/// update wheel visuals
	for (int i = 0; i < m_vehicle->getNumTires(); i++)
	{
		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		if (renderer)
		{
			btTransform tr = m_vehicle->getTire(i).m_worldTransform;
			btVector3 pos = tr.getOrigin();
			btQuaternion orn = tr.getRotation();
			renderer->writeSingleInstanceTransformToCPU(pos, orn, m_tireRenderInstances[i]);
		}
	}

	m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
	m_guiHelper->render(m_dynamicsWorld);
}

void VehicleSim::physicsDebugDraw(int debugFlags)
{
	if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
	{
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		m_dynamicsWorld->debugDrawWorld();
	}
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
	bool handled = false;
	bool isShiftPressed = m_guiHelper->getAppInterface()->m_window->isModifierKeyPressed(B3G_SHIFT);

	if (state)
	{
		if (isShiftPressed)
		{
			switch (key)
			{
			}
		}
		else
		{
			switch (key)
			{
			case B3G_LEFT_ARROW:
			{
				handled = true;
				currentSteeringAngle = defaultSteeringAngle;
				if (currentSteeringAngle > defaultSteeringAngle)
				{
					currentSteeringAngle = defaultSteeringAngle;
				}
				break;
			}
			case B3G_RIGHT_ARROW:
			{
				handled = true;
				currentSteeringAngle = -defaultSteeringAngle;
				if (currentSteeringAngle < -defaultSteeringAngle)
				{
					currentSteeringAngle = -defaultSteeringAngle;
				}
				break;
			}
			case B3G_UP_ARROW:
			{
				handled = true;
				currentEngineForce = maxEngineForce;
				currentBrakingForce = 0.f;
				break;
			}
			case B3G_DOWN_ARROW:
			{
				handled = true;
				currentEngineForce = 0.;
				currentBrakingForce = maxBrakingForce;
				break;
			}
			default:
				break;
			}
		}
	}
	else
	{
		switch (key)
		{
		case B3G_UP_ARROW:
		{
			currentEngineForce = 0.f;
			currentBrakingForce = defaultBrakingForce;
			handled = true;
			break;
		}
		case B3G_DOWN_ARROW:
		{
			currentEngineForce = 0.f;
			currentBrakingForce = defaultBrakingForce;
			handled = true;
			break;
		}
		case B3G_LEFT_ARROW:
		case B3G_RIGHT_ARROW:
		{
			currentSteeringAngle = 0.f;
			handled = true;
			break;
		}
		default:

			break;
		}
	}
	return handled;
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

///////////////////////////////////////////////////////////////////////////////
// LargeMesh
#include "../../../external/bullet/examples/Benchmarks/landscapeData.h"

int LandscapeVtxCount[] =
	{ Landscape01VtxCount, Landscape02VtxCount, Landscape03VtxCount, Landscape04VtxCount, Landscape05VtxCount,
	  Landscape06VtxCount, Landscape07VtxCount, Landscape08VtxCount, };

int LandscapeIdxCount[] =
	{ Landscape01IdxCount, Landscape02IdxCount, Landscape03IdxCount, Landscape04IdxCount, Landscape05IdxCount,
	  Landscape06IdxCount, Landscape07IdxCount, Landscape08IdxCount, };

btScalar* LandscapeVtx[] =
	{ Landscape01Vtx, Landscape02Vtx, Landscape03Vtx, Landscape04Vtx, Landscape05Vtx, Landscape06Vtx, Landscape07Vtx,
	  Landscape08Vtx, };

btScalar* LandscapeNml[] =
	{ Landscape01Nml, Landscape02Nml, Landscape03Nml, Landscape04Nml, Landscape05Nml, Landscape06Nml, Landscape07Nml,
	  Landscape08Nml, };

btScalar* LandscapeTex[] =
	{ Landscape01Tex, Landscape02Tex, Landscape03Tex, Landscape04Tex, Landscape05Tex, Landscape06Tex, Landscape07Tex,
	  Landscape08Tex, };

unsigned short* LandscapeIdx[] =
	{ Landscape01Idx, Landscape02Idx, Landscape03Idx, Landscape04Idx, Landscape05Idx, Landscape06Idx, Landscape07Idx,
	  Landscape08Idx, };

void VehicleSim::initGroundTerrain(int option)
{
	switch (option)
	{
	case 0:
	{
		btBoxShape* ground = new btBoxShape(btVector3(0.5f * terrainExtent, 0.5f * terrainThickness, 0.5f * terrainExtent));
		m_collisionShapes.push_back(ground);

		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, -5, 0));

		btRigidBody* groundRigidBody = localCreateRigidBody(0, tr, ground);

		m_guiHelper->createCollisionShapeGraphicsObject(ground);

		m_guiHelper->createCollisionObjectGraphicsObject(groundRigidBody, terrainColor);
		break;
	}
	case 1:
	{
		btTransform trans;
		trans.setIdentity();

		for (int i = 0; i < 8; i++)
		{
			btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
			btIndexedMesh part;

			part.m_vertexBase = (const unsigned char*)LandscapeVtx[i];
			part.m_vertexStride = sizeof(btScalar) * 3;
			part.m_numVertices = LandscapeVtxCount[i];
			part.m_triangleIndexBase = (const unsigned char*)LandscapeIdx[i];
			part.m_triangleIndexStride = sizeof(short) * 3;
			part.m_numTriangles = LandscapeIdxCount[i] / 3;
			part.m_indexType = PHY_SHORT;

			meshInterface->addIndexedMesh(part, PHY_SHORT);

			bool useQuantizedAabbCompression = true;
			btBvhTriangleMeshShape* triMeshShape = new btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression);
			triMeshShape->setLocalScaling(btVector3(1.0, 0.2, 1.0));
			trans.setOrigin(btVector3(0, -10, 0));

			btRigidBody* body = localCreateRigidBody(0, trans, triMeshShape);
			body->setFriction(btScalar(0.9));

			m_guiHelper->createCollisionShapeGraphicsObject(body->getCollisionShape());
			m_guiHelper->createCollisionObjectGraphicsObject(body, terrainColor);
		}
		break;
	}
	default:
		break;
	}
}

void VehicleSim::setCameraTargetPosition(float x, float y, float z)
{
	CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
	CommonCameraInterface* camera = renderer->getActiveCamera();
	camera->setCameraTargetPosition(x, y, z);
}

void VehicleSim::resetVehicle(btVector3 position)
{
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(position);
	m_vehicleChassis->setCenterOfMassTransform(tr);
	m_vehicleChassis->setAngularVelocity(btVector3(0,0,0));
	m_vehicleChassis->setLinearVelocity(btVector3(0,0,0));
}
void VehicleSim::updateGraphics()
{
	CommonExampleInterface::updateGraphics();
	btVector3 vehiclePosition = m_vehicle->getChassisWorldTransform().getOrigin();
	setCameraTargetPosition(vehiclePosition.x(), vehiclePosition.y(), vehiclePosition.z());
}
