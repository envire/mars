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

#ifndef MARS_PLUGINS_SIMMOTORCREATOR_H
#define MARS_PLUGINS_SIMMOTORCREATOR_H

#ifdef _PRINT_HEADER_
  #warning "SimMotorCreator.h"
#endif

#include <string>

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/MARSDefs.h>

#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>

#include <mars/interfaces/MotorData.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphTypes.hpp>
#include <envire_core/items/Transform.hpp>
#include <envire_core/items/Item.hpp>

#include <smurf/Robot.hpp>

#include <configmaps/ConfigData.h>

namespace mars {
  namespace plugins {
    namespace EnvireSmurfLoader {

        class SimMotorCreator 
        {
        protected:
            mars::interfaces::ControlCenter *control;
            envire::core::FrameId origin_frame_id;
            std::string type_name;

        public:
            SimMotorCreator(mars::interfaces::ControlCenter *control, envire::core::FrameId origin_frame_id)
                : control(control), origin_frame_id(origin_frame_id), type_name("smurf::Motor")
            {}

            void create(envire::core::EnvireGraph::vertex_iterator v_itr) 
            {
                envire::core::FrameId frame_id = control->graph->getFrameId(*v_itr);

                using Item = envire::core::Item<smurf::Motor>;
                using ItemItr = envire::core::EnvireGraph::ItemIterator<Item>;

                const std::pair<ItemItr, ItemItr> pair = control->graph->getItems<Item>(*v_itr);
#ifdef DEBUG
                if (pair.first == pair.second) {
                    LOG_DEBUG(("[SimMotorCreator::create] No " + type_name + " was found").c_str());
                }
#endif                 
                ItemItr i_itr;
                for(i_itr = pair.first; i_itr != pair.second; i_itr++)
                {
                    const smurf::Motor &item_data = i_itr->getData();

#ifdef DEBUG
                    LOG_DEBUG(("[SimMotorCreator::create] " + type_name + " ***" + item_data.getName() + "*** was found" ).c_str());
#endif

                    configmaps::ConfigMap motorMap = item_data.getMotorMap();    
                    //motorMap["mapIndex"].push_back(configmaps::ConfigItem(motorIndex)); // Maybe we don't need this
                    mars::interfaces::MotorData motor_data;
                    std::string prefix = "";
                    bool valid = motor_data.fromConfigMap(&motorMap, prefix, control->loadCenter);

                    if (!valid){
                        LOG_ERROR("[SimMotorCreator::create] Reading MotorData from ConfigMap failed");
                        return;
                    }

                    std::shared_ptr<mars::interfaces::MotorData> motorPtr(&motor_data);
                    unsigned long newId = control->motors->addMotor(motorPtr.get());
                    if (!newId){
                        LOG_ERROR("[SimMotorCreator::create] Add Motor into Simulation failed");
                        return;
                    }
                    else{
#ifdef DEBUG
                        LOG_DEBUG("[SimMotorCreator::create] New Motor was added into simulation");
#endif

                        // NOTE Save the data of the instantiated motor in the graph
                        using motorItemPtr = envire::core::Item<std::shared_ptr<mars::interfaces::MotorData>>::Ptr;
                        motorItemPtr motorItem(new envire::core::Item<std::shared_ptr<mars::interfaces::MotorData>>(motorPtr));
                        control->graph->addItemToFrame(frame_id, motorItem);
#ifdef DEBUG
                        LOG_DEBUG(("[SimMotorCreator::create] Motor is added into the Graph***" + frame_id + "***").c_str());
#endif                        
                    }
                }         
            }
        };

    } // end of namespace EnvireSmurfLoader
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_ENVIRESMURFLOADER
