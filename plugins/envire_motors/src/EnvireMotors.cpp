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

#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/LoadCenter.h>

#include <configmaps/ConfigData.h>
#include <base/Logging.hpp>

using namespace mars::plugins::envire_motors;
using namespace mars::utils;
using namespace mars::interfaces;
using namespace envire::core;

EnvireMotors::EnvireMotors(lib_manager::LibManager *theManager)
: MarsPluginTemplate(theManager, "EnvireMotors") {
}

void EnvireMotors::init() {
    assert(control->graph != nullptr);
    GraphEventDispatcher::subscribe(control->graph);
    GraphItemEventDispatcher<Item<smurf::Motor>>::subscribe(control->graph);
    motorIndex = 1;
}

void EnvireMotors::reset() {
}

EnvireMotors::~EnvireMotors() {
}

void EnvireMotors::itemAdded(const TypedItemAddedEvent<Item<smurf::Motor>>& e)
{
    // FIXME This method can fail if a motor tries to connect to a joint which has not been instantiated in simulation. We need dependencies as we needed for the joints.
    if (debug) 
    {
        LOG_DEBUG(("[EnvireMotors::ItemAdded] Smurf::Motor Detected in frame ***" + e.frame + "***").c_str());
    }
    smurf::Motor motorSmurf = e.item->getData();
    configmaps::ConfigMap motorMap = motorSmurf.getMotorMap();    
    //motorMap["mapIndex"].push_back(configmaps::ConfigItem(motorIndex)); // Maybe we don't need this
    mars::interfaces::MotorData motorData;
    std::string prefix = "";
    bool valid = motorData.fromConfigMap(&motorMap, prefix, control->loadCenter);
    if (!valid){
        LOG_ERROR("Reading motor map failed");
    }
    if (debug) 
    {
        LOG_DEBUG("[EnvireMotors::ItemAdded] motor max speed: %f", motorData.maxSpeed);
        LOG_DEBUG(("[EnvireMotors::ItemAdded] motor name in the map " + static_cast<std::string>(motorMap["name"])).c_str());
    }
    std::shared_ptr<mars::interfaces::MotorData> motorPtr(&motorData);
    ////bool valid = motorPtr->fromConfigMap(&motorMap, prefix, control->loadCenter);
    ////if (debug) {LOG_DEBUG(("[EnvireMotors::ItemAdded] motor name: " + motorPtr->name).c_str());}
    ////if (!valid){
    ////    LOG_ERROR("Reading motor map failed");
    ////}
    //unsigned long oldId = motorPtr->index;
    // I would have expected this oldId to be the one that is taken from the motorSmurf
    // TODO You must make sure before you instantiate the motor that the joint is already created. We can do a dependency list again.
    unsigned long newId = control->motors->addMotor(motorPtr.get());
    if (debug) {LOG_DEBUG("[EnvireMotors::ItemAdded] NewId: %d", newId);}
    if (!newId){
        LOG_ERROR("addMotor returned 0");
    }
    else{
        // NOTE Save the data of the instantiated motor in the graph
        using motorItemPtr = Item<std::shared_ptr<MotorData>>::Ptr;
        motorItemPtr motorItem(new Item<std::shared_ptr<MotorData>>(motorPtr));
        control->graph->addItemToFrame(e.frame, motorItem);
    }
    //control->loadCenter->setMappedID(oldId, newId, MAP_TYPE_MOTOR, motorIndex);
    //motorIndex += 1;
    if (debug) {LOG_DEBUG(("[EnvireMotors::ItemAdded] Smurf::Motor - Instantiated the marsMotor in frame ***" + e.frame + "***").c_str());}
    //
    //if (debug) {LOG_DEBUG("[EnvireMotors::ItemAdded] Motor index is %d", motorIndex);}
    
}

void EnvireMotors::update(sReal time_ms) {
    //control->motors->setMotorValue(1, 0.15);
    // control->motors->setMotorValue(id, value);
}

DESTROY_LIB(mars::plugins::envire_motors::EnvireMotors);
CREATE_LIB(mars::plugins::envire_motors::EnvireMotors);
