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

//NOTE We assume that there is no more than one transformation static or dynamic starting from one link/frame
// FIXME: When introducing inertial values in the nodes to join, the fixed joint brings them together
//TODO: Item Added Node interface We only allow a joint allocated in each frame, therefore we don't need a for loop here 
          
namespace mars {
  namespace plugins {
    namespace envire_joints {


      struct JointRecord
      {
          std::string name;
          std::shared_ptr<smurf::Joint> smurf;
          std::shared_ptr<mars::interfaces::JointInterface> interface;
          std::shared_ptr<mars::sim::SimJoint> sim;
      };

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
         * The first frame that is added is the root (for now)
         */
        void frameAdded(const envire::core::FrameAddedEvent& e);
        /*
         * When a new physical object is added it is checked if there is any 
         * joint missing that physical object to be generated. This is checked
         * using the dependencies map.
         */
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<std::shared_ptr<mars::interfaces::NodeInterface>>>& e);
        /*
         * If all the dependencies to instantiate the joint in the simulator are met, the joints is instantated, otherwise the missing dependencies get tracked into the staticDependencies map
         */
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::StaticTransformation>>& e);
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Joint>>& e);

      private:        


        /*
         * Create and store the struct in which the information about this joint
         * will be stored
         */
        void createJointRecord(smurf::Joint *smurfJoint, const envire::core::FrameId &frame);
        /*
         * If the joint can be instantiated then instantiates it, else add the missing dependencies.
         * 
         */
        template <class jointType>
        void checkAndInstantiate(jointType smurfJoint, bool addDeps=true);
        /*
         * Returns true if the shared_ptrs of the source and target simulation objects are in the graph.
         * (Used in checkAndInstantiate)
         */
        bool instantiable(smurf::Transformation* smurfJoint, std::shared_ptr< mars::interfaces::NodeInterface >& sourceSim, std::shared_ptr< mars::interfaces::NodeInterface >& targetSim);
        /*
         * Checks if the there is a physical object created for the given frameName
         * (Used in instantiable)
         */
        bool getSimObject(const envire::core::FrameId& frameName, std::shared_ptr<mars::interfaces::NodeInterface>& objectSim);
        /*
         * Prepares the jointData and commands the instantiation of the simulated joint (using the method join)
         * The position of the simulated joint (anchor) is given by the position of the target frame, which corresponds to the origin field of the joint in the urdf.
         * and commands the storage of the created joint
         */
        template <class jointType>
        void instantiate(jointType smurfJoint, const std::shared_ptr<mars::interfaces::NodeInterface> & sourceSim, const std::shared_ptr< mars::interfaces::NodeInterface >& targetSim);      
        /*
         * Gets the joint type (fixed) for static transformations. This method was need to implement intantiate as templated method
         */
        mars::interfaces::JointType getJointType(smurf::StaticTransformation* joint){return interfaces::JOINT_TYPE_FIXED;}
        /*
         * Gets the joint type from the smurf joint it reads it from the urdf object that it contains
         */
        mars::interfaces::JointType getJointType(smurf::Joint* joint);
        /*
         * Sets the axis1 value of the jointData for static transformations. This method was need to implement intantiate as templated method
         */
        void setAxis1(smurf::StaticTransformation* smurfJoint, mars::interfaces::JointData* jointData);
        /*
         * Sets the axis1 value of the jointData for dynamic transformations, this is read from the smurf::Joint object
         */
        void setAxis1(smurf::Joint* smurfJoint, mars::interfaces::JointData* jointData);
        /*
         * Creates the physical simulation of the joint used by instantiate 
         */
        std::shared_ptr<mars::interfaces::JointInterface> join(mars::interfaces::JointData* jointData, const std::shared_ptr< mars::interfaces::NodeInterface >& sourceSim, const std::shared_ptr< mars::interfaces::NodeInterface >& targetSim);
        /*
         * Create the simJoint from the control center and the jointData
         * Make a shared_pointer directed to the simjoint and store the shared_ptr in the graph
         * Store also the physicsJoint in the graph
         */
        void storeSimJoint(const std::shared_ptr< mars::interfaces::JointInterface >& jointInterface, mars::interfaces::JointData* jointData, envire::core::FrameId storageFrame);
        /*
         * Adds to the joint record the simjoint and joint interface pointers
         *
         */
        void addToJointRecord(const envire::core::FrameId &frameId, const std::string &jointName, mars::sim::SimJoint *simJoint, mars::interfaces::JointInterface *jointInterface);
        /*
         * @storageFrame is the frame in which the joint is stored to be recovered when the dependencies are met
         */
        void addDependencies(smurf::Transformation* smurfJoint, std::shared_ptr<mars::interfaces::NodeInterface>& sourceSim, std::shared_ptr<mars::interfaces::NodeInterface>& targetSim);
        /*
         * Attributes
         * 
         * NOTE: The second element could be just an element. We only allow a joint per frame
         * @Dependencies is used to keep track of the missing physics nodes required to create the joints. The first element of the list is the frame in which the joint having a dependency is and the second element a vector of the frames which are missing.
         */
        std::map<envire::core::FrameId, std::vector<envire::core::FrameId>> dependencies;
        bool debug = true;
        envire::core::FrameId originId;
        
      }; // end of class definition EnvireJoints

    } // end of namespace envire_joints
  } // end of namespace plugins
} // end of namespace mars
