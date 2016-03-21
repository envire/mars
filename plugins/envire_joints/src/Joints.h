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
#include <smurf/Robot.hpp>
#include <mars/interfaces/sim/NodeInterface.h>
#include <mars/interfaces/JointData.h>
#include <mars/interfaces/sim/JointInterface.h>
#include <mars/sim/SimJoint.h>
#include <mars/sim/JointPhysics.h>

namespace mars {
  namespace plugins {
    namespace envire_joints {

      class EnvireJoints: public mars::interfaces::MarsPluginTemplate,
                          public envire::core::GraphEventDispatcher,
                          public envire::core::GraphItemEventDispatcher<envire::core::Item<std::shared_ptr<mars::interfaces::NodeInterface>>>,
                          public envire::core::GraphItemEventDispatcher<envire::core::Item<smurf::StaticTransformation>>,
                          public envire::core::GraphItemEventDispatcher<envire::core::Item<smurf::Joint>>
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
        void update(mars::interfaces::sReal time_ms);        
        /*
         * When a new physical object is added it is checked if there is any 
         * joint missing that physical object to be generated. This is checked
         * using the dependencies map.
         */
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<std::shared_ptr<mars::interfaces::NodeInterface>>>& e);
        //void itemAdded(const envire::core::TypedItemAddedEvent<mars::sim::JointConfigMapItem>& e);
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::StaticTransformation>>& e);
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Joint>>& e);

      private:
        
        // Static Joints
        bool getSimObject(const envire::core::FrameId& frameName, std::shared_ptr<mars::interfaces::NodeInterface>& objectSim);
        unsigned long getSimObjectId(const envire::core::FrameId& frameName);
        bool instantiable(smurf::Transformation* smurfJoint, std::shared_ptr< mars::interfaces::NodeInterface >& sourceSim, std::shared_ptr< mars::interfaces::NodeInterface >& targetSim);
        void addDependencies(smurf::Transformation* smurfJoint, std::shared_ptr<mars::interfaces::NodeInterface>& sourceSim, std::shared_ptr<mars::interfaces::NodeInterface>& targetSim, envire::core::FrameId storageFrame);
        void storeSimJoint(const std::shared_ptr< mars::interfaces::JointInterface >& jointInterface, mars::interfaces::JointData* jointData, envire::core::FrameId storageFrame);
        void join(mars::interfaces::JointData* jointData, const std::shared_ptr< mars::interfaces::NodeInterface >& sourceSim, const std::shared_ptr< mars::interfaces::NodeInterface >& targetSim, envire::core::FrameId storageFrame);
        void instantiate(smurf::StaticTransformation* smurfJoint, const std::shared_ptr< mars::interfaces::NodeInterface >& sourceSim, const std::shared_ptr< mars::interfaces::NodeInterface >& targetSim, envire::core::FrameId storageFrame);       
        void instantiate(smurf::Joint* smurfJoint, const std::shared_ptr< mars::interfaces::NodeInterface >& sourceSim, const std::shared_ptr< mars::interfaces::NodeInterface >& targetSim, envire::core::FrameId storageFrame);       
        mars::interfaces::JointType getJointType(smurf::Joint* joint);
        //template <class jointType>
        //bool instantiateDependents (jointType joint, const envire::core::FrameId& frame);
        template <class jointType>
        void checkAndInstantiate(jointType smurfJoint, const envire::core::FrameId storageFrame, bool addDeps=true);
        
        //void addDependencies(std::vector< std::string > missingObjects, smurf::Joint dependentJoint);
        //std::shared_ptr<mars::interfaces::NodeInterface> getPhysicsInterface(const std::string& frameName);
          
        //bool instantiable(smurf::Joint smurfTf);
        std::map<envire::core::FrameId, std::vector<envire::core::FrameId>> dependencies;
        
        
        bool debug = false;
        
      }; // end of class definition EnvireJoints

    } // end of namespace envire_joints
  } // end of namespace plugins
} // end of namespace mars
