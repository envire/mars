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
 * \file EnvireMls.h
 * \author Raul (Raul.Dominguez@dfki.de)
 * \brief Provides
 *
 * Version 0.1
 */

#pragma once

#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>

#include <string>

#include <envire_core/graph/EnvireGraph.hpp>

namespace mars {

  namespace plugins {
    namespace envire_mls {

      // inherit from MarsPluginTemplateGUI for extending the gui
      class EnvireMls: public mars::interfaces::MarsPluginTemplate {

      public:
        EnvireMls(lib_manager::LibManager *theManager);
        ~EnvireMls();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("envire_mls"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);

        // EnvireMls methods
        void addMLS(envire::core::FrameId center, const std::string & mlsPath);

      private:


      }; // end of class definition EnvireMls

    } // end of namespace envire_mls
  } // end of namespace plugins
} // end of namespace mars

