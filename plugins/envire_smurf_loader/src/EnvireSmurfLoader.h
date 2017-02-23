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

#ifndef MARS_PLUGINS_ENVIRESMURFLOADER_H
#define MARS_PLUGINS_ENVIRESMURFLOADER_H

#ifdef _PRINT_HEADER_
  #warning "EnvireSmurfLoader.h"
#endif

#include <string>

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/LoadSceneInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>

#include <mars/interfaces/NodeData.h>

#include <mars/sim/JointRecord.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphTypes.hpp>
#include <envire_core/items/Transform.hpp>

#include <smurf/Robot.hpp>

namespace mars {
  namespace plugins {
    namespace EnvireSmurfLoader {

#define DEBUG

      // inherit from MarsPluginTemplateGUI for extending the gui
      class EnvireSmurfLoader: public mars::interfaces::LoadSceneInterface {


      public:
        EnvireSmurfLoader(lib_manager::LibManager *theManager);
        ~EnvireSmurfLoader();

        envire::core::GraphTraits::vertex_descriptor addCenter();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("eniver_smurf_loader"); }
        CREATE_MODULE_INFO();

        virtual bool loadFile(std::string filename, std::string tmpPath,
                                std::string robotname);

        virtual bool loadFile(std::string filename, std::string tmpPath,
                                std::string robotname, utils::Vector pos, utils::Vector rot);

        virtual int saveFile(std::string filename, std::string tmpPath);

      private:
        interfaces::ControlCenter *control;

        void addFloor(const envire::core::GraphTraits::vertex_descriptor &center);

        void addRobot(std::string filename,  envire::core::GraphTraits::vertex_descriptor center, envire::core::Transform iniPose);

        int nextGroupId;

        void createSimObjects();

        void loadNodes();
        void loadJoints();

        bool getSimObject(const envire::core::FrameId& frameName, std::shared_ptr<mars::sim::SimNode>& objectSim);

        template <class ItemDataType>
        void loadJoint(envire::core::EnvireGraph::vertex_iterator v_itr, std::string type_name);

        envire::core::FrameId center;

      }; // end of class definition EnvireSmurfLoader

    } // end of namespace EnvireSmurfLoader
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_ENVIRESMURFLOADER
