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
 * \file EnvireMotors.cpp
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Create
 *
 * Version 0.1
 */


#include "EnvireMotors.hpp"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

#include <envire_core/graph/EnvireGraph.hpp>


using namespace mars::plugins::envire_motors;
using namespace mars::utils;
using namespace mars::interfaces;

EnvireMotors::EnvireMotors(lib_manager::LibManager *theManager)
: MarsPluginTemplate(theManager, "EnvireMotors") {
}

void EnvireMotors::init() {
    assert(control->graph != nullptr);
    GraphEventDispatcher::subscribe(control->graph);
    GraphItemEventDispatcher<envire::core::Item<smurf::Motor>>::subscribe(control->graph);
}

void EnvireMotors::reset() {
}

EnvireMotors::~EnvireMotors() {
}

void EnvireMotors::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Motor>>& e)
{
    if (debug) {LOG_DEBUG("[EnvireMotors::ItemAdded] XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");}
    smurf::Motor motor = e.item->getData();
    mars::interfaces::MotorData marsMotor = motor.getMarsMotor();
    if (debug) {LOG_DEBUG(("[EnvireMotors::ItemAdded] Smurf::Motor - Instantiated the marsMotor in frame ***" + e.frame + "***").c_str());}
}

void EnvireMotors::update(sReal time_ms) {
    
    // control->motors->setMotorValue(id, value);
}

DESTROY_LIB(mars::plugins::envire_motors::EnvireMotors);
CREATE_LIB(mars::plugins::envire_motors::EnvireMotors);
