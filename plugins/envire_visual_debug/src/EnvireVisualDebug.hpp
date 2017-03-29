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
 * \file EnvireVisualDebug.h
 * \author Raul (Raul.Dominguez@dfki.de)
 * \brief Debug
 *
 * Version 0.1
 */

#pragma once

#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/cfg_manager/CFGManagerInterface.h>

#include <envire_visualizer/EnvireVisualizerWindow.hpp>

#include <string>

namespace mars {

  namespace plugins {
    namespace envire_visual_debug {

      // inherit from MarsPluginTemplateGUI for extending the gui
      class EnvireVisualDebug: public mars::interfaces::MarsPluginTemplateGUI,
        // for gui
        public mars::main_gui::MenuInterface,
        public mars::cfg_manager::CFGClient {

      public:
        EnvireVisualDebug(lib_manager::LibManager *theManager);
        ~EnvireVisualDebug();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("envire_visual_debug"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);

      private:
          
        double timeSinceLastUiUpdate;
        // MenuInterface methods
        void menuAction(int action, bool checked = false);

        envire::viz::EnvireVisualizerWindow *graphWindow;

      }; // end of class definition EnvireVisualDebug




    } // end of namespace visual
  } // end of namespace plugins
} // end of namespace mars

