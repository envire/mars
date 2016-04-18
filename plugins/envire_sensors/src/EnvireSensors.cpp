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
#include <mars/interfaces/sim/NodeInterface.h>
#include <mars/sim/SimNode.h>

#include <mars/sim/RotatingRaySensor.h>
#include <mars/sim/NodePositionSensor.h>
#include <mars/sim/NodeCOMSensor.h>

#include <base/Time.hpp>
#include <configmaps/ConfigData.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <base/samples/Pointcloud.hpp>


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


      void EnvireSensors::display_position_data(const std::shared_ptr< BaseSensor >& sensorPtr)
      {
        // TODO For this sensor I m not doing anything with the broadcaster. How is the new data arriving to the object?
        std::string name = sensorPtr->getName();
        LOG_DEBUG(("[EnvireSensor::display_position_data] We found the sensor with name: "+ name).c_str());
        
        mars::sim::NodePositionSensor * posSensor;
        posSensor = dynamic_cast<mars::sim::NodePositionSensor*>(sensorPtr.get());
        LOG_DEBUG(("[EnvireSensor::display_position_data] We have the position sensor with name: " + posSensor->getName()).c_str());
        
        sReal *sens_val;
        int count_val = posSensor->getSensorData(&sens_val);
        
        //char * data;
        //int dimensions = posSensor-> getAsciiData(data);
        
        //sReal **data;
        //int dimensions = posSensor->getSensorData(data);
        
        LOG_DEBUG("[EnvireSensor::display_position_data] Dimensions of the sensor data: , %d ", count_val);
        LOG_DEBUG("[EnvireSensor::display_position_data] data 1: , %d", ((double)sens_val[0]));
        LOG_DEBUG("[EnvireSensor::display_position_data] data 2: , %d", ((double)sens_val[1]));
        LOG_DEBUG("[EnvireSensor::display_position_data] data 3: , %d", ((double)sens_val[2]));
      }
      
      void EnvireSensors::rotate_rotatingRay_sensor(const std::shared_ptr<BaseSensor>& sensorPtr)
      {
        mars::sim::RotatingRaySensor * raySensor;
        raySensor = dynamic_cast<mars::sim::RotatingRaySensor*>(sensorPtr.get());
        utils::Quaternion offset = raySensor->turn();
      }
            
      void EnvireSensors::display_rotatingRay_data(const std::shared_ptr< BaseSensor >& sensorPtr)
      {
        mars::sim::RotatingRaySensor * raySensor;
        raySensor = dynamic_cast<mars::sim::RotatingRaySensor*>(sensorPtr.get());
        utils::Quaternion offset = raySensor->turn();
        
        base::samples::Pointcloud pointcloud;
        pointcloud.time = base::Time::now();
        
        std::vector<mars::utils::Vector> data;
        if(raySensor->getPointcloud(data)) { //NOTE This returns an empty pointcloud unless the sensor has finnished the turn
          std::vector<mars::utils::Vector>::iterator it = data.begin();
          for(; it != data.end(); it++) {
            base::Vector3d vec((*it)[0], (*it)[1], (*it)[2]);
            pointcloud.points.push_back(vec);
          }
        }
        if (pointcloud.points.size() > 0)
        {
          LOG_DEBUG("[EnvireSensor::display_rotatingRay_data] We have a pointcloud with %d points", pointcloud.points.size());
        }
      }
      
      void EnvireSensors::display_COM_data(const std::shared_ptr<BaseSensor>& sensorPtr)
      {
        mars::sim::NodeCOMSensor * comSensor;
        comSensor = dynamic_cast<mars::sim::NodeCOMSensor* >(sensorPtr.get());
        
        sReal *sens_val;
        int count_val = comSensor->getSensorData(&sens_val);
        
        LOG_DEBUG("[enviresensor::display_contact_data] dimensions of the sensor data: , %d ", count_val);
        LOG_DEBUG("[enviresensor::display_contact_data] data 1: , %6.2f", ((double)sens_val[0]));
        LOG_DEBUG("[enviresensor::display_contact_data] data 2: , %6.2f", ((double)sens_val[1]));
        LOG_DEBUG("[enviresensor::display_contact_data] data 3: , %6.2f", ((double)sens_val[2]));
      }
        
      void EnvireSensors::updateSimNode(){
        // Update the SimNode
        using SimNodeItem = Item<std::shared_ptr<mars::sim::SimNode>>;
        using SimNodeIterator = EnvireGraph::ItemIterator<SimNodeItem>;
        std::shared_ptr<mars::sim::SimNode> simNodePtr;
        SimNodeIterator begin, end;
        boost::tie(begin, end) = control->graph->getItems<SimNodeItem>("velodyne_link");
        if (begin != end){
          simNodePtr = begin->getData();
          simNodePtr->update(0.0); //NOTE I need to provide a calc_ms. I provide 0.
        }
      }
      
      void EnvireSensors::update(sReal time_ms) {
        updateSimNode();
        using sensorItem = envire::core::Item<std::shared_ptr<BaseSensor>>;
        using Iterator = envire::core::EnvireGraph::ItemIterator<sensorItem>;
        std::shared_ptr<BaseSensor> sensorPtr;
        Iterator begin, end;
        boost::tie(begin, end) = control->graph->getItems<sensorItem>("velodyne_link");
        if (begin != end){
            sensorPtr = begin->getData();
            rotate_rotatingRay_sensor(sensorPtr);
            //display_position_data(sensorPtr);
            //display_rotatingRay_data(sensorPtr);
            //display_COM_data(sensorPtr);
        }
      }

      void EnvireSensors::itemAdded(const TypedItemAddedEvent<Item<smurf::Sensor>>& e)
      {
        // NOTE Sensor instantiation based on smurf.cpp - addConfigMap() and 
        // FIXME This method fails if a sensor tries to connect to a link which has not been instantiated in simulation. We need dependencies as we needed for the joints.
        if(debug) 
        {
          LOG_DEBUG(("[EnvireSensors::ItemAdded] Smurf::Sensor Detected in frame ***" + e.frame + "***").c_str());
        }
        smurf::Sensor sensorSmurf = e.item->getData();
        configmaps::ConfigMap sensorMap = sensorSmurf.getMap();
        sensorMap["frame"] = e.frame;
        if ((std::string) sensorMap["type"] == "Joint6DOF") {
          std::string linkname = (std::string) sensorMap["link"];
          // the name of the joint to which is attached
          // in smurf.cpp:
          //std::string jointname = model->getlink(linkname)->parent_joint->name;
          std::string jointname = e.frame;
          if (debug)
          {
            LOG_DEBUG(("[EnvireSensors::ItemAdded] linkname "+ linkname).c_str());
            LOG_DEBUG(("[EnvireSensors::ItemAdded] jointname "+ jointname).c_str());
          }
          //NOTE I think we don't need the following things (from smurf.cpp): 
          //sensorMap["nodeID"] = (ulong) nodeIDMap[linkname];
          //sensorMap["jointID"] = (ulong) jointIDMap[jointname];
          //fprintf(stderr, "creating Joint6DOF..., %lu, %lu\n", (ulong) sensorMap["nodeID"], (ulong) sensorMap["jointID"]);
        }
        //NOTE More stuff is done in addConfigMap for the sensors, but I think we don't need it in the envire_graph, essentially mapping to the nodes or joints, which we have already because we stored in the correspondent frame the sensor
        BaseSensor* sensor = new BaseSensor;
        sensor = control->sensors->createAndAddSensor(&sensorMap); 
        
        // NOTE Store the new sensor in the Envire Graph.
        // TODO Store the sensor with the specific type it is, instead of as a Base Sensor
        using SensorItemPtr = envire::core::Item<std::shared_ptr<BaseSensor>>::Ptr;
        SensorItemPtr sensorItem(new envire::core::Item<std::shared_ptr<BaseSensor>>(sensor));
        control->graph->addItemToFrame(e.frame, sensorItem);
        LOG_DEBUG("[EnvireSensors::ItemAdded] Base sensor instantiated and addedto the graph. Now it will be attached to a node");
        
        // NOTE Attach the sensor to the simNode that corresponds (the one in the same frame by  now)
        using SimNodeItem = Item<std::shared_ptr<mars::sim::SimNode>>;
        using Iterator = EnvireGraph::ItemIterator<SimNodeItem>;
        std::shared_ptr<mars::sim::SimNode> simNodePtr;
        Iterator begin, end;
        boost::tie(begin, end) = control->graph->getItems<SimNodeItem>(e.frame);
        if (begin != end){
          simNodePtr = begin->getData();
          simNodePtr->addSensor(sensor);
          if (debug) {
               LOG_DEBUG("[EnvireSensors::ItemAdded] The SimNode to attach the sensor is found");
          }
        }
        else
        {
          LOG_ERROR("[EnvireSensors::ItemAdded] Could not find node interface to which to attach the sensor. ");       
        }
        
        // Here might be the reason why it does not work: First Look, everything seems to be fine here
        // Maybe the problem is ...
        // - Due to data Broker
        // - Wrong sensor initialization
        // - ...?
        // TODO SimNode AddSensor method seems not to show traces, is it not executin?
        //simNode->addSensor(sensor);
      }
    } // end of namespace envire_sensors
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_sensors::EnvireSensors);
CREATE_LIB(mars::plugins::envire_sensors::EnvireSensors);
