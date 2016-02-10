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
 * \file SMURFToSimulation.h
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Tests
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_SMURF2SIMULATION_H
#define MARS_PLUGINS_SMURF2SIMULATION_H

#ifdef _PRINT_HEADER_
  #warning "SMURFToSimulation.h"
#endif

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <mars/interfaces/sim/ControlCenter.h> 

#include <envire_smurf/Robot.hpp>
#include <envire_core/graph/GraphTypes.hpp>
#include <string>

namespace mars {

  namespace plugins {
    namespace SMURFToSimulation {

      // inherit from MarsPluginTemplateGUI for extending the gui
      class SMURFToSimulation: public mars::interfaces::MarsPluginTemplate,
        public mars::data_broker::ReceiverInterface,
        // for gui
        // public mars::main_gui::MenuInterface,
        public mars::cfg_manager::CFGClient {


      public:
        SMURFToSimulation(lib_manager::LibManager *theManager);
        ~SMURFToSimulation();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("SMURFToSimulation"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);

        // DataBrokerReceiver methods
        virtual void receiveData(const data_broker::DataInfo &info,
                                 const data_broker::DataPackage &package,
                                 int callbackParam);
        // CFGClient methods
        virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);

        // MenuInterface methods
        //void menuAction(int action, bool checked = false);

        // SMURFToSimulation methods

      private:
        cfg_manager::cfgPropertyStruct example;
        envire::core::GraphTraits::vertex_descriptor addFloor();
        void addRobot(envire::core::GraphTraits::vertex_descriptor center);
        int nextGroupId;

      }; // end of class definition SMURFToSimulation

    } // end of namespace SMURFToSimulation
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_SMURF2SIMULATION
