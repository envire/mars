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
 * \file EnvireJoints.cpp
 * \author Raul.Dominguez (Raul.Dominguez@dfki.de)
 * \brief Create
 *
 * Version 0.1
 */


#include "Joints.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

#include <envire_core/graph/TransformGraph.hpp>
#include <urdf_model/model.h>
#include <mars/interfaces/JointData.h>

namespace mars {
  namespace plugins {
    namespace envire_joints {
      
      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace envire::core;
      
      EnvireJoints::EnvireJoints(lib_manager::LibManager *theManager)
      : MarsPluginTemplate(theManager, "EnvireJoints"), 
      GraphEventDispatcher(){
      }
      
      void EnvireJoints::init() {
        assert(control->graph != nullptr);
        GraphEventDispatcher::subscribe(control->graph); // ASK: one needs the header of the TransformGraph?
        GraphItemEventDispatcher<Item<smurf::Joint>::Ptr>::subscribe(control->graph);
        LOG_DEBUG("[Envire Joints] Init Method");
      }
      
      void EnvireJoints::reset() {
      }
      
      std::shared_ptr<mars::interfaces::NodeInterface> EnvireJoints::getPhysicsInterface(const std::string& frameName){
        using physicsItemPtr = Item<std::shared_ptr<mars::interfaces::NodeInterface>>::Ptr;
        using Iterator = TransformGraph::ItemIterator<physicsItemPtr>;
        Iterator begin, end;
        LOG_DEBUG("[Envire Joints]::getPhysicsInterface: Frame name is "+frameName);
        boost::tie(begin, end) = control->graph->getItems<physicsItemPtr>(frameName);
        std::shared_ptr<mars::interfaces::NodeInterface> result;
        if (begin != end){
          result = (*begin)->getData();
        }
        else{
          LOG_ERROR("[Envire Joints]::getPhysicsInterface: A joint misses target or source");
        }
        return result;
      }
        
      /*
       *  Mars Joint Types
       * // Definition of Joint Types
       * enum JointType {
       *   JOINT_TYPE_UNDEFINED=0,
       *   JOINT_TYPE_HINGE,
       *   JOINT_TYPE_HINGE2,
       *   JOINT_TYPE_SLIDER,
       *   JOINT_TYPE_BALL,
       *   JOINT_TYPE_UNIVERSAL,
       *   JOINT_TYPE_FIXED,
       *   JOINT_TYPE_ISTRUCT_SPINE,
       *   NUMBER_OF_JOINT_TYPES
       *   };
       * 
       * 
       */
      void EnvireJoints::itemAdded(const TypedItemAddedEvent<Item<smurf::Joint>::Ptr>& e){
        using namespace mars::interfaces;
        // The type of joints correspondences is taken from mars::smurf::SMURF::handleKinematics
        LOG_DEBUG("[Envire Joints] A joint was added to the graph");
        // See what type of joint is and generate the correspondent simulated one
        smurf::Joint smurfJoint = e.item->getData();
        boost::shared_ptr<urdf::Joint> jointModel = smurfJoint.getJointModel();
        std::string logType;
        JointType jointType;
        switch (jointModel->type)
        {
          case urdf::Joint::FIXED:
          {
            logType = "Fixed";
            jointType = JOINT_TYPE_FIXED; // use consts so that all are defined only once
            break;
          }
          case urdf::Joint::FLOATING:
          {
            //TODO We have some but seem not to be supported by mars?  Ask
            logType = "Floating";
            jointType = JOINT_TYPE_FIXED;
            break;
          }
          case urdf::Joint::CONTINUOUS:
          {
            // We have some
            logType = "Continuous"; 
            jointType = JOINT_TYPE_HINGE;
            break;
          }
          case urdf::Joint::PRISMATIC:
          {
            logType = "Prismatic"; 
            jointType = JOINT_TYPE_SLIDER;
            break;
          }
          case urdf::Joint::REVOLUTE:
          {
            // We have some
            logType = "Revolute"; 
            jointType = JOINT_TYPE_HINGE;
            break;
          }
          case urdf::Joint::PLANAR:
          {
            logType = "Planar"; 
            // TODO No support? Set fixed
            jointType = JOINT_TYPE_FIXED;
            break;
          }
          case urdf::Joint::UNKNOWN:
          default:
          {
            logType = "Unknown"; 
            jointType = JOINT_TYPE_FIXED;
            break;
          }
          
        }
        LOG_DEBUG("[Envire Joints] The joint type is: " + logType); 
        std::string jointName = smurfJoint.getName();
        // We now need the node id for the source and target object to bind, which corresponds to the shared_ptr of the simulated objects
        std::string sourceName = smurfJoint.getSourceFrame().getName();
        std::string targetName = smurfJoint.getTargetFrame().getName();
        LOG_DEBUG("[Envire Joints] The joint target is: " + targetName); 
        std::shared_ptr<NodeInterface> source = getPhysicsInterface(sourceName);
        std::shared_ptr<NodeInterface> target = getPhysicsInterface(targetName);
        /*
         *      shared_ptr<JointInterface> jointInterface(PhysicsMapper::newJointPhysics(control->sim->getPhysics()));
         *      // create the physical node data
         *      LOG_DEBUG("[Envire Physics] Just before creating the physical joint");
         *      if(jointInterface->createJoint(&jointPhysics, node1.get(), node2.get()))
         *      {
         *  LOG_DEBUG("[Envire Physics] Just after creating the physical joint");
         *        //remember a pointer to the interface, otherwise it will be deleted.
         *        uuidToJoints[jointID] = jointInterface;
         *        control->sim->sceneHasChanged(false);//important, otherwise the joint will be ignored by simulation
          }
          else
          {
          std::cerr << "ERROR: Failed to create joint" << std::endl;
          assert(false);//this only happens if the JointConfigMapItem contains bad data, which it should never do
          }
          
          }
          */
        
        //control->graph->addItemToFrame(e.frame, physicsItem);
        
        //JointData marsJoint(jointName, jointType, uuidSource, uuidTarget);
        // We still need the Physicsuuid which is stored in the physics plugin...
      }
      
      void EnvireJoints::update(sReal time_ms) {
        
      }
      
    } // end of namespace envire_joints
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_joints::EnvireJoints);
CREATE_LIB(mars::plugins::envire_joints::EnvireJoints);
