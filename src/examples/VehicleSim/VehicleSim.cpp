/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///September 2006: VehicleDemo is work in progress, this file is mostly just a placeholder
///This VehicleDemo file is very early in development, please check it later
///A function that maps user input (throttle) into torque/force applied on the wheels
///with gears etc.

/*
 * March 2022
 * Michael Dodd altered original version.
 * Vehicle simulation on uneven terrain modeling
 * -wheel friction
 * -suspension
 */
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"

class btVehicleTuning;

struct btVehicleRaycaster;

class btCollisionShape;

#include <iostream>
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "../../physics/CommonRigidBodyBase.h"

#include "../../../external/bullet/examples/Benchmarks/landscapeData.h"
#include "../CommonExampleInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "../../graphics/CommonGUIHelperInterface.h"
#include "../../graphics/CommonRenderInterface.h"
#include "../../graphics/CommonWindowInterface.h"
#include "../../graphics/CommonGraphicsAppInterface.h"

///VehicleDemo shows how to setup and use the built-in raycast vehicle
class VehicleSim : public CommonExampleInterface
{
 public:
	GUIHelperInterface* m_guiHelper;

	/* extra stuff*/
	btVector3 m_cameraPosition;

	class btDiscreteDynamicsWorld* m_dynamicsWorld;

	btDiscreteDynamicsWorld* getDynamicsWorld()
	{
		return m_dynamicsWorld;
	}

	btRigidBody* m_carChassis;

	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

	int m_wheelInstances[4];

	bool m_useDefaultCamera;

	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface* m_overlappingPairCache;

	class btCollisionDispatcher* m_dispatcher;

	class btConstraintSolver* m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	class btTriangleIndexVertexArray* m_indexVertexArrays;

	btVector3* m_vertices;

	btRaycastVehicle::btVehicleTuning m_tuning;
	btVehicleRaycaster* m_vehicleRayCaster;
	btRaycastVehicle* m_vehicle;
	btCollisionShape* m_wheelShape;

	float m_cameraHeight;

	float m_minCameraDistance;
	float m_maxCameraDistance;

	VehicleSim(struct GUIHelperInterface* helper);

	virtual ~VehicleSim();

	virtual void stepSimulation(float deltaTime);

	virtual void resetVehicle();

	virtual void clientResetScene();

	virtual void displayCallback();

	virtual void createLargeMeshBody();

	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}

	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}

	virtual bool keyboardCallback(int key, int state);

	virtual void renderScene();

	virtual void physicsDebugDraw(int debugFlags);

	void setCameraPosition(float x, float y, float z);

	void UpdateHUD();

	void initPhysics();

	void exitPhysics();

	virtual void resetCamera()
	{
		float dist = 8;
		float pitch = -32;
		float yaw = -45;
		float targetPos[3] = { -0.33, -0.72, 4.5 };
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

btScalar maxMotorImpulse = 4000.f;

//the sequential impulse solver has difficulties dealing with large mass ratios (differences), between loadMass and the fork parts
btScalar loadMass = 350.f;  //
//btScalar loadMass = 10.f;//this should work fine for the SI solver

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4 0.785398163397448309616
#endif

int rightIndex = 0;
int upIndex = 1;
int forwardIndex = 2;
btVector3 wheelDirectionCS0(0, -1, 0);
btVector3 wheelAxleCS(-1, 0, 0);

bool useMCLPSolver = true;

#include <stdio.h>  //printf debugging

#include "VehicleSim.h"

///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts

// vehicle body dimensions [m]
float bodyWidth = 1.9f;
float bodyHeight = 1.8f;
float bodyLength = 3.5f;

float gEngineForce = 0.f;

float defaultBreakingForce = 10.f;
float maxBreakingForce = 100.f;
float gBreakingForce = 0.f;

float maxEngineForce = 1000.f;  //this should be engine/velocity dependent

float gVehicleSteering = 0.f;
float steeringIncrement = 0.04f;
float steeringClamp = 0.3f;
float wheelHeight = 0.8f;
float wheelRadius = 0.35f;
float wheelWidth = 0.3f;
float wheelBaseFront = 2.1f;
float wheelBaseRear = wheelBaseFront;
float wheelFriction = 1;
float suspensionStiffness = 20.f;
float suspensionDamping = 2.3f;
float suspensionCompression = 4.4f;
float rollInfluence = 0.1f;  //1.0f;

// whether to render the wheels as boxes instead of cylinders
bool renderWheelsAsBoxes = false;

btVector4 chassisColor{ 72. / 256., 133. / 256., 237. / 256., 1 };
btVector4 terrainColor{ 112. / 256., 129. / 256., 87. / 256., 1 };

btScalar suspensionRestLength(0.6);

////////////////////////////////////

VehicleSim::VehicleSim(struct GUIHelperInterface* helper)
	: m_guiHelper(helper),
	  m_carChassis(0),
	  m_indexVertexArrays(0),
	  m_vertices(0),
	  m_cameraHeight(4.f),
	  m_minCameraDistance(3.f),
	  m_maxCameraDistance(10.f)
{
	helper->setUpAxis(1);
	m_vehicle = 0;
	m_wheelShape = 0;
	m_cameraPosition = btVector3(30, 30, 30);
	m_useDefaultCamera = false;
}

void VehicleSim::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
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

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_indexVertexArrays;
	delete m_vertices;

	//delete dynamics world
	delete m_dynamicsWorld;
	m_dynamicsWorld = 0;

	delete m_vehicleRayCaster;
	m_vehicleRayCaster = 0;

	delete m_vehicle;
	m_vehicle = 0;

	delete m_wheelShape;
	m_wheelShape = 0;

	//delete solver
	delete m_constraintSolver;
	m_constraintSolver = 0;

	//delete broadphase
	delete m_overlappingPairCache;
	m_overlappingPairCache = 0;

	//delete dispatcher
	delete m_dispatcher;
	m_dispatcher = 0;

	delete m_collisionConfiguration;
	m_collisionConfiguration = 0;
}

VehicleSim::~VehicleSim()
{
	//exitPhysics();
}

///////////////////////////////////////////////////////////////////////////////
// LargeMesh

int LandscapeVtxCount[] = {
	Landscape01VtxCount,
	Landscape02VtxCount,
	Landscape03VtxCount,
	Landscape04VtxCount,
	Landscape05VtxCount,
	Landscape06VtxCount,
	Landscape07VtxCount,
	Landscape08VtxCount,
};

int LandscapeIdxCount[] = {
	Landscape01IdxCount,
	Landscape02IdxCount,
	Landscape03IdxCount,
	Landscape04IdxCount,
	Landscape05IdxCount,
	Landscape06IdxCount,
	Landscape07IdxCount,
	Landscape08IdxCount,
};

btScalar* LandscapeVtx[] = {
	Landscape01Vtx,
	Landscape02Vtx,
	Landscape03Vtx,
	Landscape04Vtx,
	Landscape05Vtx,
	Landscape06Vtx,
	Landscape07Vtx,
	Landscape08Vtx,
};

btScalar* LandscapeNml[] = {
	Landscape01Nml,
	Landscape02Nml,
	Landscape03Nml,
	Landscape04Nml,
	Landscape05Nml,
	Landscape06Nml,
	Landscape07Nml,
	Landscape08Nml,
};

btScalar* LandscapeTex[] = {
	Landscape01Tex,
	Landscape02Tex,
	Landscape03Tex,
	Landscape04Tex,
	Landscape05Tex,
	Landscape06Tex,
	Landscape07Tex,
	Landscape08Tex,
};

unsigned short* LandscapeIdx[] = {
	Landscape01Idx,
	Landscape02Idx,
	Landscape03Idx,
	Landscape04Idx,
	Landscape05Idx,
	Landscape06Idx,
	Landscape07Idx,
	Landscape08Idx,
};

void VehicleSim::createLargeMeshBody()
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
		btVector3 localInertia(0, 0, 0);
		trans.setOrigin(btVector3(0, -10, 0));

		btRigidBody* body = localCreateRigidBody(0, trans, triMeshShape);
		body->setFriction(btScalar(0.9));

		m_guiHelper->createCollisionShapeGraphicsObject(body->getCollisionShape());
		m_guiHelper->createCollisionObjectGraphicsObject(body, terrainColor);
	}
}

void VehicleSim::initPhysics()
{
	int upAxis = 1;

	m_guiHelper->setUpAxis(upAxis);

//    btVector3 groundExtents(50, 50, 50);
//    groundExtents[upAxis] = 3;
//    btCollisionShape* groundShape = new btBoxShape(groundExtents);
//    m_collisionShapes.push_back(groundShape);

	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000, -1000, -1000);
	btVector3 worldMax(1000, 1000, 1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin, worldMax);
	if (useMCLPSolver)
	{
		btDantzigSolver* mlcp = new btDantzigSolver();
		//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
		btMLCPSolver* sol = new btMLCPSolver(mlcp);
		m_constraintSolver = sol;
	}
	else
	{
		m_constraintSolver = new btSequentialImpulseConstraintSolver();
	}
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver,
		m_collisionConfiguration);
	if (useMCLPSolver)
	{
		m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize =
			1;  //for direct solver it is better to have a small A matrix
	}
	else
	{
		m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize =
			128;  //for direct solver, it is better to solve multiple objects together, small batches have high overhead
	}
	m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.00001;

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, -3, 0));

	//create ground object
//    localCreateRigidBody(0, tr, groundShape);
	createLargeMeshBody();


	// Chassis coordinate system
	// +x - left
	// +y - up
	// +z - forward
	btCollisionShape* chassisShape = new btBoxShape(btVector3(0.5f * bodyWidth, 0.5f * bodyHeight, 0.5f * bodyLength));
	m_collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, bodyHeight, 0));

	compound->addChildShape(localTrans, chassisShape);

	tr.setOrigin(btVector3(0, 0.f, 0));

	// vehicle mass [kg]
	float vehicleMass = 700.0f;
	m_carChassis = localCreateRigidBody(vehicleMass, tr, compound);  //chassisShape);

	if (renderWheelsAsBoxes)
	{
		m_wheelShape = new btBoxShape(btVector3(0.5f * wheelWidth, wheelRadius, wheelRadius));
	}
	else
	{
		m_wheelShape = new btCylinderShapeX(btVector3(0.5f * wheelWidth, wheelRadius, wheelRadius));
	}

	m_guiHelper->createCollisionShapeGraphicsObject(m_wheelShape);
	int wheelGraphicsIndex = m_wheelShape->getUserIndex();

	const float position[4] = { 0, 10, 10, 0 };
	const float quaternion[4] = { 0, 0, 0, 1 };
	const float color[4] = { 0.2, 0.2, 0.2, 1 };
	const float scaling[4] = { 1, 1, 1, 1 };

	for (int i = 0; i < 4; i++)
	{
		m_wheelInstances[i] = m_guiHelper->registerGraphicsInstance(wheelGraphicsIndex, position, quaternion, color,
			scaling);
	}

	/// create vehicle
	{
		m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
		m_vehicle = new btRaycastVehicle(m_tuning, m_carChassis, m_vehicleRayCaster);

		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);

		m_dynamicsWorld->addVehicle(m_vehicle);

		bool isFrontWheel = true;

		//choose coordinate system
		m_vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

		const float halfBodyLength = bodyLength * 0.5f;

		// left front
		btVector3 connectionPointCS0(0.5f * wheelBaseFront, wheelHeight, halfBodyLength - wheelRadius);

		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius,
			m_tuning, isFrontWheel);

		// right front
		connectionPointCS0 = btVector3(-0.5f * wheelBaseFront, wheelHeight, halfBodyLength - wheelRadius);

		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius,
			m_tuning, isFrontWheel);

		// right rear
		connectionPointCS0 = btVector3(-0.5f * wheelBaseRear, wheelHeight,
			-halfBodyLength + wheelRadius);
		isFrontWheel = false;
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius,
			m_tuning, isFrontWheel);

		// left rear
		connectionPointCS0 = btVector3(0.5f * wheelBaseRear, wheelHeight, -halfBodyLength + wheelRadius);
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius,
			m_tuning, isFrontWheel);

		for (int i = 0; i < m_vehicle->getNumWheels(); i++)
		{
			btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
	}

	resetVehicle();

	m_guiHelper->createCollisionShapeGraphicsObject(m_carChassis->getCollisionShape());
	m_guiHelper->createCollisionObjectGraphicsObject(m_carChassis, chassisColor);
}

void VehicleSim::physicsDebugDraw(int debugFlags)
{
	if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
	{
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		m_dynamicsWorld->debugDrawWorld();
	}
}

//to be implemented by the demo
void VehicleSim::renderScene()
{

	for (int i = 0; i < m_vehicle->getNumWheels(); i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i, true);

		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		if (renderer)
		{
			btTransform tr = m_vehicle->getWheelInfo(i).m_worldTransform;
			btVector3 pos = tr.getOrigin();
			btQuaternion orn = tr.getRotation();
			renderer->writeSingleInstanceTransformToCPU(pos, orn, m_wheelInstances[i]);
		}
	}

	m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
	m_guiHelper->render(m_dynamicsWorld);

	ATTRIBUTE_ALIGNED16(btScalar)
		m[16];
	int i;

	btVector3 worldBoundsMin, worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin, worldBoundsMax);

	for (i = 0; i < m_vehicle->getNumWheels(); i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i, true);
		//draw wheels (cylinders)
		m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
	}
}

void VehicleSim::stepSimulation(float deltaTime)
{
	btVector3 vehiclePosition = m_vehicle->getChassisWorldTransform().getOrigin();
	setCameraPosition(vehiclePosition.x(), vehiclePosition.y(), vehiclePosition.z());
	UpdateHUD();
	{
		// rear wheels
		int wheelIndex = 2;
		m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
		m_vehicle->setBrake(gBreakingForce, wheelIndex);
		wheelIndex = 3;
		m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
		m_vehicle->setBrake(gBreakingForce, wheelIndex);

		// front wheels
		wheelIndex = 0;
		m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
		wheelIndex = 1;
		m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
	}

	float dt = deltaTime;

	if (m_dynamicsWorld)
	{
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = 2;

		int numSimSteps;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt, maxSimSubSteps);

		if (m_dynamicsWorld->getConstraintSolver()->getSolverType() == BT_MLCP_SOLVER)
		{
			btMLCPSolver* sol = (btMLCPSolver*)m_dynamicsWorld->getConstraintSolver();
			int numFallbacks = sol->getNumFallbacks();
			if (numFallbacks)
			{
				static int totalFailures = 0;
				totalFailures += numFallbacks;
				printf("MLCP solver failed %d times, falling back to btSequentialImpulseSolver (SI)\n", totalFailures);
			}
			sol->setNumFallbacks(0);
		}

//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n", numSimSteps - maxSimSubSteps, numSimSteps);
			}
			else
			{
				printf("Simulated (%i) steps\n", numSimSteps);
			}
		}
#endif  //VERBOSE_FEEDBACK
	}
}

void VehicleSim::displayCallback(void)
{
	//optional but useful: debug drawing
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->debugDrawWorld();
	}
}

void VehicleSim::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void VehicleSim::resetVehicle()
{
	gVehicleSteering = 0.f;
	gBreakingForce = defaultBreakingForce;
	gEngineForce = 0.f;

	m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_carChassis->setLinearVelocity(btVector3(0, 0, 0));
	m_carChassis->setAngularVelocity(btVector3(0, 0, 0));
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
		m_carChassis->getBroadphaseHandle(), getDynamicsWorld()->getDispatcher());
	if (m_vehicle)
	{
		m_vehicle->resetSuspension();
		for (int i = 0; i < m_vehicle->getNumWheels(); i++)
		{
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_vehicle->updateWheelTransform(i, true);
		}
	}
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
				gVehicleSteering += steeringIncrement;
				gVehicleSteering = steeringClamp;
				if (gVehicleSteering > steeringClamp)
					gVehicleSteering = steeringClamp;

				break;
			}
			case B3G_RIGHT_ARROW:
			{
				handled = true;
				gVehicleSteering -= steeringIncrement;
				gVehicleSteering = -steeringClamp;
				if (gVehicleSteering < -steeringClamp)
					gVehicleSteering = -steeringClamp;

				break;
			}
			case B3G_UP_ARROW:
			{
				handled = true;
				gEngineForce = maxEngineForce;
				gBreakingForce = 0.f;
				break;
			}
			case B3G_DOWN_ARROW:
			{
				handled = true;
				gEngineForce = 0.;
				gBreakingForce = maxBreakingForce;
				break;
			}

			case B3G_F7:
			{
				handled = true;
				btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*)m_dynamicsWorld;
				world->setLatencyMotionStateInterpolation(!world->getLatencyMotionStateInterpolation());
				printf("world latencyMotionStateInterpolation = %d\n", world->getLatencyMotionStateInterpolation());
				break;
			}
			case B3G_F6:
			{
				handled = true;
				//switch solver (needs demo restart)
				useMCLPSolver = !useMCLPSolver;
				printf("switching to useMLCPSolver = %d\n", useMCLPSolver);

				delete m_constraintSolver;
				if (useMCLPSolver)
				{
					btDantzigSolver* mlcp = new btDantzigSolver();
					//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
					btMLCPSolver* sol = new btMLCPSolver(mlcp);
					m_constraintSolver = sol;
				}
				else
				{
					m_constraintSolver = new btSequentialImpulseConstraintSolver();
				}

				m_dynamicsWorld->setConstraintSolver(m_constraintSolver);

				break;
			}

			case B3G_F5:
				handled = true;
				m_useDefaultCamera = !m_useDefaultCamera;
				break;
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
			gEngineForce = 0.f;
			gBreakingForce = defaultBreakingForce;
			handled = true;
			break;
		}
		case B3G_DOWN_ARROW:
		{
			gEngineForce = 0.f;
			gBreakingForce = defaultBreakingForce;
			handled = true;
			break;
		}
		case B3G_LEFT_ARROW:
		case B3G_RIGHT_ARROW:
		{
			gVehicleSteering = 0.f;
			handled = true;
			break;
		}
		default:

			break;
		}
	}
	return handled;
}

btRigidBody*
VehicleSim::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
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

void VehicleSim::setCameraPosition(float x, float y, float z)
{
	CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
	CommonCameraInterface* camera = renderer->getActiveCamera();
	camera->setCameraTargetPosition(x, y, z);
}

void VehicleSim::UpdateHUD()
{
	float yellow[4] = { 0, 0, 0, 1 };

	char speed[120];
	sprintf(speed, "Speed: %4.1f km/h", m_vehicle->getCurrentSpeedKmHour());
	m_guiHelper->getAppInterface()->drawText(speed, 10, 1000, 1, yellow);
}

CommonExampleInterface* VehicleSimCreateFunc(struct CommonExampleOptions& options)
{
	return new VehicleSim(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(VehicleSimCreateFunc)
