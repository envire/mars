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
 * \file Visualizer.h
 * \author Raul__author__Arne (raul.dominguez@dfki.de)
 * \brief visualizer
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_VISUALIZER_H
#define MARS_PLUGINS_VISUALIZER_H

#ifdef _PRINT_HEADER_
  #warning "Visualizer.h"
#endif

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/graphics/GraphicsUpdateInterface.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <envire_core/EnvireVisualizerWindow.hpp>
#include <string>

namespace mars {

  namespace plugins {
    namespace visualizer {

      // inherit from MarsPluginTemplateGUI for extending the gui
      class Visualizer: public mars::interfaces::MarsPluginTemplate,
        public mars::data_broker::ReceiverInterface,
        public mars::interfaces::GraphicsUpdateInterface,
        public mars::cfg_manager::CFGClient {

      public:
        Visualizer(lib_manager::LibManager *theManager);
        ~Visualizer();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("visualizer"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);
        
        virtual void preGraphicsUpdate(void);

        // DataBrokerReceiver methods
        virtual void receiveData(const data_broker::DataInfo &info,
                                 const data_broker::DataPackage &package,
                                 int callbackParam);
        // CFGClient methods
        virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);

        // MenuInterface methods
        //void menuAction(int action, bool checked = false);

        // Visualizer methods

      private:
        envire::viz::EnvireVisualizerWindow window;
        cfg_manager::cfgPropertyStruct example;
        bool visualizerInitialized;

      }; // end of class definition Visualizer

    } // end of namespace visualizer
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_VISUALIZER_H
