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
 * \file EnvireSmurfLoader.h
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Tests
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_SIMSENSORCREATOR_H
#define MARS_PLUGINS_SIMSENSORCREATOR_H

#ifdef _PRINT_HEADER_
  #warning "SimSensorCreator.h"
#endif

#include <string>

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/MARSDefs.h>

#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

#include <mars/sim/RotatingRaySensor.h>
#include <mars/sim/NodePositionSensor.h>
#include <mars/sim/NodeCOMSensor.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphTypes.hpp>
#include <envire_core/items/Transform.hpp>
#include <envire_core/items/Item.hpp>

#include <smurf/Robot.hpp>

#include <configmaps/ConfigData.h>

namespace mars {
  namespace plugins {
    namespace EnvireSmurfLoader {

        class SimSensorCreator 
        {
        protected:
            mars::interfaces::ControlCenter *control;
            envire::core::FrameId origin_frame_id;
            std::string type_name;

        public:
            SimSensorCreator(mars::interfaces::ControlCenter *control, envire::core::FrameId origin_frame_id)
                : control(control), origin_frame_id(origin_frame_id), type_name("smurf::Sensor")
            {}

            void create(envire::core::EnvireGraph::vertex_iterator v_itr) 
            {
                envire::core::FrameId frame_id = control->graph->getFrameId(*v_itr);

                using Item = envire::core::Item<smurf::Sensor>;
                using ItemItr = envire::core::EnvireGraph::ItemIterator<Item>;

                const std::pair<ItemItr, ItemItr> pair = control->graph->getItems<Item>(*v_itr);
#ifdef DEBUG
                if (pair.first == pair.second) {
                    LOG_DEBUG(("[SimSensorCreator::create] No " + type_name + " was found").c_str());
                }
#endif                 
                ItemItr i_itr;
                for(i_itr = pair.first; i_itr != pair.second; i_itr++)
                {
                    const smurf::Sensor &item_data = i_itr->getData();


                    using SensorItemPtr = envire::core::Item<std::shared_ptr<mars::interfaces::BaseSensor>>::Ptr;

                    if (item_data.getType() == "RotatingRaySensor")
                    {
#ifdef DEBUG
                        LOG_DEBUG(("[SimSensorCreator::create] " + type_name + " RotatingRaySensor ***" + item_data.getName() + "*** was found" ).c_str());
#endif

                        std::shared_ptr<mars::interfaces::BaseSensor> sensor(createSensor(item_data, frame_id));

                        SensorItemPtr sensorItem(new envire::core::Item<std::shared_ptr<mars::interfaces::BaseSensor>>(sensor));
                        control->graph->addItemToFrame(frame_id, sensorItem);
                        
#ifdef DEBUG
                        LOG_DEBUG("[SimSensorCreator::create] Base sensor instantiated and added to the graph.");
#endif

                        bool attached = attachSensor(sensor.get(), frame_id);
                        if (!attached)
                        {              
                            LOG_ERROR(("[SimSensorCreator::create] Could not find node interface to which to attach the sensor *" + item_data.getName() + "*.").c_str());
                        } else {
                            LOG_DEBUG(("[SimSensorCreator::create] *" + item_data.getType() + "* *" + item_data.getName() + "* is attached (frame: " + frame_id + ")").c_str());
                        }
                    }
                    else if (item_data.getType() == "Joint6DOF")
                    {
#ifdef DEBUG
                        LOG_DEBUG(("[SimSensorCreator::create] " + type_name + " Joint6DOF ***" + item_data.getName() + "*** was found" ).c_str());
#endif

                        std::shared_ptr<mars::interfaces::BaseSensor> sensor(createSensor(item_data, frame_id));

                        SensorItemPtr sensorItem(new envire::core::Item<std::shared_ptr<mars::interfaces::BaseSensor>>(sensor));
                        control->graph->addItemToFrame(frame_id, sensorItem);

#ifdef DEBUG
                        LOG_DEBUG("[SimSensorCreator::create] Base sensor instantiated and added to the graph.");
#endif

                        bool attached = attachSensor(sensor.get(), frame_id);
                        if (!attached)
                        {
                            LOG_ERROR(("[SimSensorCreator::create] Could not find node interface to which to attach the sensor *" + item_data.getName() + "*.").c_str());
                        } else 
                        {
#ifdef DEBUG              
                            LOG_DEBUG(("[SimSensorCreator::create] *" + item_data.getType() + "* *" + item_data.getName() + "* is attached (frame: " + frame_id + ")").c_str());
#endif
                        }
                    }
                    else if (item_data.getType() == "CameraSensor")
                    {
#ifdef DEBUG
                        LOG_DEBUG(("[SimSensorCreator::create] A Camera sensor  has been found with name " + item_data.getName()).c_str());
#endif
                        if(item_data.getMap().hasKey("depthImage"))
                        {
                            if(item_data.getMap()["depthImage"])
                            {
                                std::shared_ptr<mars::interfaces::BaseSensor> sensor(createSensor(item_data, frame_id));
                                SensorItemPtr sensorItem(new envire::core::Item<std::shared_ptr<mars::interfaces::BaseSensor>>(sensor));
                                control->graph->addItemToFrame(frame_id, sensorItem);
#ifdef DEBUG
                                LOG_DEBUG("[SimSensorCreator::create] Base sensor instantiated and addedto the graph.");
#endif
                                bool attached = attachSensor(sensor.get(), frame_id);
                                                                
                                if (!attached)
                                {              
                                    LOG_ERROR("[SimSensorCreator::create] Could not find node interface to which to attach the sensor *" + item_data.getName() + "*.");
                                } else {
                                    LOG_DEBUG(("[SimSensorCreator::create] *" + item_data.getType() + "* *" + item_data.getName() + "* is attached (frame: " + frame_id + ")").c_str());
                                }
                            }
                            
                        }
                        
                    }
                    else
                    {
#ifdef DEBUG
                        LOG_DEBUG(("[SimSensorCreator::create] " + type_name + " UnknownType ***" + item_data.getName() + "*** was found" ).c_str());
#endif
                    }
                }         
            }

            mars::interfaces::BaseSensor* createSensor(const smurf::Sensor &sensorSmurf, const envire::core::FrameId frameId)
            {
                configmaps::ConfigMap sensorMap = sensorSmurf.getMap();
                sensorMap["frame"] = frameId;
                mars::interfaces::BaseSensor* sensor = control->sensors->createAndAddSensor(&sensorMap); 
                return sensor;
            }   

            bool attachSensor(mars::interfaces::BaseSensor* sensor, const envire::core::FrameId frameId)
            {
                using SimNodeItem = envire::core::Item<std::shared_ptr<mars::sim::SimNode>>;
                using Iterator = envire::core::EnvireGraph::ItemIterator<SimNodeItem>;
                bool attached = false;
                std::shared_ptr<mars::sim::SimNode> simNodePtr;
                Iterator begin, end;
                boost::tie(begin, end) = control->graph->getItems<SimNodeItem>(frameId);
                if (begin != end)
                {
                    simNodePtr = begin->getData();
                    simNodePtr->addSensor(sensor);
                    attached = true;
#ifdef DEBUG
                    LOG_DEBUG(("[SimSensorCreator::attachSensor] The SimNode ***" + simNodePtr->getName() + "*** to attach the sensor is found").c_str()); 
#endif
                }
                return attached;
            }                     
        };

    } // end of namespace EnvireSmurfLoader
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_ENVIRESMURFLOADER
