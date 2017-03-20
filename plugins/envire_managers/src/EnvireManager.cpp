/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the MARS simulation framework.
 *
 *  MARS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation, either version 3
 *  of the License, or (at your option) any later version.
 *
 *  MARS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with MARS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * \file EnvireManager.cpp
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Create
 *
 * Version 0.1
 */


#include "EnvireManager.hpp"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

#include <envire_core/graph/EnvireGraph.hpp>

#include <mars/interfaces/sim/LoadCenter.h>

#include <configmaps/ConfigData.h>
#include <base/Logging.hpp>

#include "EnvireMotorManager.hpp"
#include "EnvireNodeManager.hpp"

// Comment-in the following line in order to get debug traces
//#define DEBUG

using namespace mars::plugins::envire_managers;
using namespace mars::utils;
using namespace mars::interfaces;
using namespace envire::core;

EnvireManager::EnvireManager(lib_manager::LibManager *theManager)
: MarsPluginTemplate(theManager, "EnvireManager") {

    LOG_INFO("[EnvireManager] set EnvireMotorManager for control->motors");
    control->nodes = new EnvireNodeManager(control, theManager);
    control->motors = new EnvireMotorManager(control);

    // create graph
    control->graph = std::shared_ptr<envire::core::EnvireGraph> (new envire::core::EnvireGraph());
    
    // init root frame
    std::string center = "center";
	control->graph->addFrame(center);
}

void EnvireManager::init() {
    assert(control->graph != nullptr);
    motorIndex = 1;
}

void EnvireManager::reset() {
}

EnvireManager::~EnvireManager() {
}

void EnvireManager::update(sReal time_ms) {
    //control->motors->setMotorValue(1, 0.15);
    // control->motors->setMotorValue(id, value);
}

DESTROY_LIB(mars::plugins::envire_managers::EnvireManager);
CREATE_LIB(mars::plugins::envire_managers::EnvireManager);
