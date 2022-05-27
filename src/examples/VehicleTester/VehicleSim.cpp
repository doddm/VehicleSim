//
// Created by mchld on 5/25/2022.
//

#include "VehicleSim.h"

void VehicleSim::Initialize()
{
	m_guiHelper->setUpAxis(1);

}
VehicleSim::VehicleSim(struct GUIHelperInterface* helper) : m_guiHelper(helper)
{
	// set y-up in GUI
	helper->setUpAxis(1);
}

