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
        GraphEventDispatcher::subscribe(control->graph.get());
        //GraphItemEventDispatcher<Item<smurf::Sensor>>::subscribe(control->graph.get());
        velodyneFrame = "velodyne";

        joint6dof_frame.push_back("leg0_ft");
        joint6dof_frame.push_back("leg1_ft");
        joint6dof_frame.push_back("leg2_ft");
        joint6dof_frame.push_back("leg3_ft");
        joint6dof_frame.push_back("leg4_ft");
        joint6dof_frame.push_back("leg5_ft");
        
      }

      void EnvireSensors::reset() {
      }

      EnvireSensors::~EnvireSensors() {
      }
        
      void EnvireSensors::updateVelodyneSim(){
        using SimNodeItem = Item<std::shared_ptr<mars::sim::SimNode>>;
        using SimNodeIterator = EnvireGraph::ItemIterator<SimNodeItem>;
        std::shared_ptr<mars::sim::SimNode> simNodePtr;
        SimNodeIterator begin, end;
        boost::tie(begin, end) = control->graph->getItems<SimNodeItem>(velodyneFrame);
        double calc_ms = control->sim->getCalcMs();
        if (begin != end){
          simNodePtr = begin->getData();
          simNodePtr->update(calc_ms); //NOTE I need to provide a calc_ms. I provide 0.
        }
      }

      void EnvireSensors::updateJoint6DOF()
      {
        using SimNodeItem = Item<std::shared_ptr<mars::sim::SimNode>>;
        using SimNodeIterator = EnvireGraph::ItemIterator<SimNodeItem>;
        std::shared_ptr<mars::sim::SimNode> simNodePtr;
        double calc_ms = control->sim->getCalcMs();
        for (int i = 0; i < joint6dof_frame.size(); ++i)
        {
            SimNodeIterator begin, end;
            boost::tie(begin, end) = control->graph->getItems<SimNodeItem>(joint6dof_frame.at(i));
            if (begin != end){
              simNodePtr = begin->getData();
              simNodePtr->update(calc_ms); //NOTE I need to provide a calc_ms. I provide 0.
            }        
        }
      }
      
      void EnvireSensors::update(sReal time_ms) {
        //if (velodyneFrame != "")
        //{
         // updateVelodyneSim();
        //}
        //if (joint6dof_frame.size() != 0)
        //{
         //   updateJoint6DOF();
        //}
      }

      void EnvireSensors::itemAdded(const TypedItemAddedEvent<Item<smurf::Sensor>>& e)
      {
        using SensorItemPtr = envire::core::Item<std::shared_ptr<BaseSensor>>::Ptr;

        smurf::Sensor smurfSensor = e.item->getData();

#ifdef DEBUG
            LOG_DEBUG(("[EnvireSensors::ItemAdded] Smurf::Sensor *" + smurfSensor.getName() + "* in frame *" + e.frame + "*").c_str());
#endif

        if (smurfSensor.getType() == "RotatingRaySensor")
        {
            velodyneFrame = e.frame; 
            std::shared_ptr<BaseSensor> sensor(createSensor(smurfSensor, e.frame));

            
            SensorItemPtr sensorItem(new envire::core::Item<std::shared_ptr<BaseSensor>>(sensor));
            control->graph->addItemToFrame(e.frame, sensorItem);
            
#ifdef DEBUG
                LOG_DEBUG("[EnvireSensors::ItemAdded] Base sensor instantiated and addedto the graph.");
#endif

            bool attached = attachSensor(sensor.get(), e.frame);
            if (!attached)
            {              
                LOG_ERROR("[EnvireSensors::ItemAdded] Could not find node interface to which to attach the sensor *" + smurfSensor.getName() + "*.");
            } else {
                LOG_DEBUG(("[EnvireSensors::ItemAdded] *" + smurfSensor.getType() + "* *" + smurfSensor.getName() + "* is attached (frame: " + e.frame + ")").c_str());
            }
        }
        else if (smurfSensor.getType() == "Joint6DOF")
        {
            joint6dof_frame.push_back(e.frame);
            std::shared_ptr<BaseSensor> sensor(createSensor(smurfSensor, e.frame));


            SensorItemPtr sensorItem(new envire::core::Item<std::shared_ptr<BaseSensor>>(sensor));
            control->graph->addItemToFrame(e.frame, sensorItem);

#ifdef DEBUG
                LOG_DEBUG("[EnvireSensors::ItemAdded] Base sensor instantiated and addedto the graph.");
#endif

            bool attached = attachSensor(sensor.get(), e.frame);
            if (!attached)
            {
                LOG_ERROR("[EnvireSensors::ItemAdded] Could not find node interface to which to attach the sensor *" + smurfSensor.getName() + "*.");
            } else 
            {
#ifdef DEBUG              
                LOG_DEBUG(("[EnvireSensors::ItemAdded] *" + smurfSensor.getType() + "* *" + smurfSensor.getName() + "* is attached (frame: " + e.frame + ")").c_str());
#endif
            }
        }
        else
        {
#ifdef DEBUG
          LOG_DEBUG(("[EnvireSensors::ItemAdded] Sensor type " + smurfSensor.getType() + " not supported.").c_str());
#endif
        }
      }

      mars::interfaces::BaseSensor* EnvireSensors::createSensor(smurf::Sensor &sensorSmurf, const envire::core::FrameId frameId)
      {
        configmaps::ConfigMap sensorMap = sensorSmurf.getMap();
        sensorMap["frame"] = frameId;
        BaseSensor* sensor = control->sensors->createAndAddSensor(&sensorMap); 
        return sensor;
      }

      bool EnvireSensors::attachSensor(mars::interfaces::BaseSensor* sensor, const envire::core::FrameId frameId)
      {
        using SimNodeItem = Item<std::shared_ptr<mars::sim::SimNode>>;
        using Iterator = EnvireGraph::ItemIterator<SimNodeItem>;
        bool attached = false;
        std::shared_ptr<mars::sim::SimNode> simNodePtr;
        Iterator begin, end;
        boost::tie(begin, end) = control->graph->getItems<SimNodeItem>(frameId);
        if (begin != end){
          simNodePtr = begin->getData();
          simNodePtr->addSensor(sensor);
          attached = true;
#ifdef DEBUG
          LOG_DEBUG("[EnvireSensors::ItemAdded] The SimNode to attach the sensor is found"); 
#endif
        }
        return attached;
      }

    } // end of namespace envire_sensors
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_sensors::EnvireSensors);
CREATE_LIB(mars::plugins::envire_sensors::EnvireSensors);

      /*
      // Just for testing:
      void EnvireSensors::update(sReal time_ms) {
        if (velodyneFrame != "")
        {
          updateVelodyneSim();
        }
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

      void EnvireSensors::rotate_rotatingRay_sensor(const std::shared_ptr<BaseSensor>& sensorPtr)
      {
        mars::sim::RotatingRaySensor * raySensor;
        raySensor = dynamic_cast<mars::sim::RotatingRaySensor*>(sensorPtr.get());
        raySensor->turn();
        //utils::Quaternion offset = raySensor->turn();
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
      

      void EnvireSensors::display_rotatingRay_data(const std::shared_ptr< BaseSensor >& sensorPtr)
      {
        mars::sim::RotatingRaySensor * raySensor;
        raySensor = dynamic_cast<mars::sim::RotatingRaySensor*>(sensorPtr.get());
        raySensor->turn();
        //utils::Quaternion offset = raySensor->turn();
        
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
    */



