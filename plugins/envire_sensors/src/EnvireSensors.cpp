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
 * \file EnvireSensors.cpp
 * \author Raul (Raul.Dominguez@dfki.de)
 * \brief sensors-plugin-envire-mars
 *
 * Version 0.1
 */


#include "EnvireSensors.hpp"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

#include <mars/interfaces/sim/SensorManagerInterface.h>

#include <envire_core/graph/EnvireGraph.hpp>

namespace mars {
  namespace plugins {
    namespace envire_sensors {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace mars::plugins::envire_sensors;
      using namespace envire::core;

      EnvireSensors::EnvireSensors(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "EnvireSensors") {
      }
  
      void EnvireSensors::init() {
        assert(control->graph != nullptr);
        GraphEventDispatcher::subscribe(control->graph);
        GraphItemEventDispatcher<Item<smurf::Sensor>>::subscribe(control->graph);
        if (debug) { LOG_DEBUG( "[EnvireSensors]::init Init method"); }
      }

      void EnvireSensors::reset() {
      }

      EnvireSensors::~EnvireSensors() {
      }


      void EnvireSensors::update(sReal time_ms) {

      }


      void EnvireSensors::itemAdded(const TypedItemAddedEvent<Item<smurf::Sensor>>& e)
      {
        // Sensor instantiation based on smurf.cpp - addConfigMap() and 
        // FIXME This method fails if a sensor tries to connect to a link which has not been instantiated in simulation. We need dependencies as we needed for the joints.
        // First will be implemented with this error case not been covered and once sensors can be simulated this will be tackled
        if(debug) 
        {
            LOG_DEBUG(("[EnvireSensors::ItemAdded] Smurf::Sensor Detected in frame ***" + e.frame + "***").c_str());
        }
        smurf::Sensor sensorSmurf = e.item->getData();
        configmaps::ConfigMap sensorMap = sensorSmurf.getMap();
        if ((std::string) sensorMap["type"] == "Joint6DOF") {
          std::string linkname = (std::string) sensorMap["link"];
          // The name of the joint to which is attached
          // in smurf.cpp:
          //std::string jointname = model->getLink(linkname)->parent_joint->name;
          std::string jointname = e.frame;
          if (debug)
          {
            LOG_DEBUG(("[EnvireSensors::ItemAdded] linkname "+ linkname).c_str());
            LOG_DEBUG(("[EnvireSensors::ItemAdded] jointname "+ jointname).c_str());
          //fprintf(stderr, "addConfig: %s\n", jointname.c_str());
          //fprintf(stderr, "addConfig: %s\n", linkname.c_str());
          }

          // I think we don't need the following things (from smurf.cpp): 
          //sensorMap["nodeID"] = (ulong) nodeIDMap[linkname];
          //sensorMap["jointID"] = (ulong) jointIDMap[jointname];
          //fprintf(stderr, "creating Joint6DOF..., %lu, %lu\n", (ulong) sensorMap["nodeID"], (ulong) sensorMap["jointID"]);
        }
        // More stuff is done in addConfigMap for the sensors, but I think we don't need it in the envire_graph
        BaseSensor* sensor = new BaseSensor;
        sensor = control->sensors->createAndAddSensor(&sensorMap); //Look into this method
        // I think that I just need to find the simNode that corresponds and attach the Sensor
        // Create the Sensor based on what is in CreateAndAddSensor
        //          Get the type and the BaseConfig // 1st CreateAndAddSensor
        //          BaseSensor *sensor = ((*it).second)(this->control,config); // 2nd CreateAndAddSensor
        //          Get the node and attach to it the BaseSensor // See NodeManager::addNodeSensor
        
        if (sensor != 0)
        {
          LOG_ERROR("[EnvireSensors::ItemAdded] Error in create sensor ");
        }
        // Save the instantiated sensor in the graph
        using sensorItemPtr = Item<std::shared_ptr<BaseSensor>>::Ptr;
        sensorItemPtr sensorItem(new Item<std::shared_ptr<BaseSensor>>(sensor));
        control->graph->addItemToFrame(e.frame, sensorItem);
      }
    } // end of namespace envire_sensors
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_sensors::EnvireSensors);
CREATE_LIB(mars::plugins::envire_sensors::EnvireSensors);
