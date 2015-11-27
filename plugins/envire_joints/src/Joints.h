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
 * \file EnvireJoints.h
 * \author Raul.Dominguez (Raul.Dominguez@dfki.de)
 * \brief Plugin
 *
 */

#pragma once

#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>

#include <string>

#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>
#include <envire_core/events/ItemAddedEvent.hpp>
#include <envire_core/items/Item.hpp>
#include <smurf/Smurf.hpp>
#include <mars/interfaces/sim/NodeInterface.h>

namespace mars {
  namespace plugins {
    namespace envire_joints {

      class EnvireJoints: public mars::interfaces::MarsPluginTemplate,
                          public envire::core::GraphEventDispatcher,
                          public envire::core::GraphItemEventDispatcher<envire::core::Item<smurf::Joint>::Ptr>      
      {

      public:
        EnvireJoints(lib_manager::LibManager *theManager);

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("envire_joints"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Joint>::Ptr>& e);
        void update(mars::interfaces::sReal time_ms);
        
        // EnvireJoints methods
        bool instantiable(smurf::Joint smurfJoint);
        void instantiate(smurf::Joint smurfJoint);
        void addDependencies(std::vector< std::string > missingObjects, smurf::Joint dependentJoint);

        std::shared_ptr<mars::interfaces::NodeInterface> getPhysicsInterface(const std::string& frameName);

        
      private:
        
        std::map<envire::core::FrameId, std::vector<smurf::Joint>> dependencies;
        
      }; // end of class definition EnvireJoints

    } // end of namespace envire_joints
  } // end of namespace plugins
} // end of namespace mars