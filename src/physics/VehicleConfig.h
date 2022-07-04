//
// Created by mchld on 7/3/2022.
//

#ifndef BULLETGAME_SRC_PHYSICS_VEHICLECONFIG_H_
#define BULLETGAME_SRC_PHYSICS_VEHICLECONFIG_H_

#include "LinearMath/btScalar.h"

struct VehicleConfigData
{
	btScalar m_bodyWidth;
	btScalar m_bodyHeight;
	btScalar m_bodyLength;
};

struct VehicleConfig
{
	VehicleConfig(VehicleConfigData& vcd)
	{
		m_bodyWidth = vcd.m_bodyWidth;
		m_bodyLength = vcd.m_bodyLength;
		m_bodyHeight = vcd.m_bodyHeight;
	}

	btScalar m_mass = 700;

	btScalar m_bodyWidth = 1.9;
	btScalar m_bodyHeight = 1.8;
	btScalar m_bodyLength = 3.5;

	btScalar m_defaultSteeringAngle = 0.3f;
	btScalar m_maxBrakingForce = 3000.0f;
	btScalar m_maxEngineForce = 1500.0f;

	btScalar m_steeringIncrement = 0.02f;
	btScalar m_wheelBaseFront = 3.0f;
	btScalar m_wheelFriction = 100;
	btScalar m_suspensionStiffness = 9.0f;
	btScalar m_suspensionDamping = 1.0f;
	btScalar m_suspensionLength = 0.8;
};

#endif //BULLETGAME_SRC_PHYSICS_VEHICLECONFIG_H_
