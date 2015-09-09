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
 * \file TestTreeMars.h
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Plugin
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_TESTTREEMARS_H
#define MARS_PLUGINS_TESTTREEMARS_H

#ifdef _PRINT_HEADER_
  #warning "TestTreeMars.h"
#endif

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <mars/sim/TreeMars.h>

#include <string>

namespace mars {
  namespace plugins {
    namespace TestTreeMars {

      // inherit from MarsPluginTemplateGUI for extending the gui
      class TestTreeMars: public mars::interfaces::MarsPluginTemplate,
        public mars::data_broker::ReceiverInterface,
        // for gui
        // public mars::main_gui::MenuInterface,
        public mars::cfg_manager::CFGClient {

      public:
        TestTreeMars(lib_manager::LibManager *theManager);
        ~TestTreeMars();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("TestTreeMars"); }
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

        // TestTreeMars methods

      private:
        void addBall(const std::string& name);//adds a ball at a random location with random weight
      private:
        cfg_manager::cfgPropertyStruct example;
        int drawingId;

      }; // end of class definition TestTreeMars

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_TESTTREEMARS_H
