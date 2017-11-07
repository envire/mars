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

#ifndef MARS_PLUGINS_ENVIREGRAPHLOADER_H
#define MARS_PLUGINS_ENVIREGRAPHLOADER_H

#ifdef _PRINT_HEADER_
  #warning "EnvireMlsLoader.hpp"
#endif

#include <string>

#include <mars/interfaces/sim/LoadSceneInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphTypes.hpp>
#include <envire_core/items/Transform.hpp>

#include <maps/grid/MLSMap.hpp>

namespace mars {
  namespace plugins {
    namespace EnvireGraphLoader {

      using mlsPrec = maps::grid::MLSMapPrecalculated;
      using mlsKal = maps::grid::MLSMapKalman;

      // inherit from MarsPluginTemplateGUI for extending the gui
      class EnvireGraphLoader: public mars::interfaces::LoadSceneInterface {


      public:
        EnvireGraphLoader(lib_manager::LibManager *theManager);
        ~EnvireGraphLoader();

        envire::core::GraphTraits::vertex_descriptor addCenter();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("envire_mls_loader"); }
        CREATE_MODULE_INFO();

        virtual bool loadFile(std::string filename, std::string tmpPath,
                                std::string robotname);

        virtual bool loadFile(std::string filename, std::string tmpPath,
                                std::string robotname, utils::Vector pos, utils::Vector rot);


        virtual int saveFile(std::string filename, std::string tmpPath);

        void loadMLSMap(const std::string & mlsPath);

      private:
        interfaces::ControlCenter *control;

        mlsPrec getMLSMap(const envire::core::EnvireGraph & graph, envire::core::FrameId mlsFrameId);

        envire::core::FrameId center;
        envire::core::FrameId mlsFrameId;

      }; // end of class definition EnvireGraphLoader

    } // end of namespace EnvireGraphLoader
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_ENVIRESGRAPHLOADER
