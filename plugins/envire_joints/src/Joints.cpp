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

#include <envire_core/graph/EnvireGraph.hpp>

#include <mars/sim/PhysicsMapper.h>

#include <urdf_model/model.h>
#include <mars/interfaces/sim/JointInterface.h>
#include <mars/interfaces/sim/NodeInterface.h>

namespace mars {
  namespace plugins {
    namespace envire_joints {
      
      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace envire::core;
      using namespace mars::sim;
      using physicsNodeItem = Item<std::shared_ptr<NodeInterface>>;
      using physicsJointItemPtr = Item<std::shared_ptr<mars::interfaces::JointInterface>>::Ptr;
      using simJointItemPtr = Item<std::shared_ptr<mars::sim::SimJoint>>::Ptr;

      // Public Methods
      EnvireJoints::EnvireJoints(lib_manager::LibManager *theManager): MarsPluginTemplate(theManager, "EnvireJoints"), GraphEventDispatcher(){
      }
      
      void EnvireJoints::init() {
        assert(control->graph != nullptr);
        GraphEventDispatcher::subscribe(control->graph);
        GraphItemEventDispatcher<Item<smurf::StaticTransformation>>::subscribe(control->graph);
        GraphItemEventDispatcher<Item<std::shared_ptr<NodeInterface>>>::subscribe(control->graph);
        GraphItemEventDispatcher<Item<smurf::Joint>>::subscribe(control->graph);
      }
      
      void EnvireJoints::reset() {
      }
      
      void EnvireJoints::update(sReal time_ms) {
      }
      
      void EnvireJoints::itemAdded(const  envire::core::TypedItemAddedEvent<envire::core::Item<std::shared_ptr<NodeInterface>>>& e){
        using dynamicTfsIterator = EnvireGraph::ItemIterator<Item<smurf::Joint>>;
        using staticTfsIterator = EnvireGraph::ItemIterator<Item<smurf::StaticTransformation>>;
        if (debug) { LOG_DEBUG( "[Envire Joints] itemAdded A new <std::shared_ptr<NodeInterface>> was added to frame '"+ e.frame+"'"); }
        std::map<FrameId, std::vector<FrameId>>::iterator iterDeps = dependencies.find(e.frame);
        if (iterDeps != dependencies.end())
        {
          if (debug) { LOG_DEBUG( "[Envire Joints] itemAdded in frame '"+ e.frame + "' is matching at least a dependency"); }
          std::vector<FrameId> dependentFrames = dependencies[e.frame];
          for(FrameId frame : dependentFrames)
          {
            staticTfsIterator beginStaticTfs, endStaticTfs;
            boost::tie(beginStaticTfs, endStaticTfs) = control->graph->getItems<Item<smurf::StaticTransformation>>(frame);
            if (beginStaticTfs != endStaticTfs)
            {
              if (debug) { LOG_DEBUG( "[Envire Joints] itemAdded in frame '"+ e.frame + "' is matching a static dependency"); }
              smurf::StaticTransformation* smurfTf = &(beginStaticTfs->getData());
              bool addDeps = false; // This avoids dependencies to be included twice
              checkAndInstantiate(smurfTf, frame, addDeps);
            }
            else
            {
              // Dynamic transformations should have their own frame, therefore they can not be together in a frame with a static one
              dynamicTfsIterator beginDynTfs, endDynTfs;
              boost::tie(beginDynTfs, endDynTfs) = control->graph->getItems<Item<smurf::Joint>>(frame);
              if (beginDynTfs != endDynTfs)
              {
                if (debug) { LOG_DEBUG( "[Envire Joints] itemAdded in frame '"+ e.frame + "' is matching a dynamic dependency"); }
                smurf::Joint * smurfTf = &(beginDynTfs->getData());
                bool addDeps = false; // This avoids dependencies to be included twice
                checkAndInstantiate(smurfTf, frame, addDeps);
              }
            }
            dependencies.erase(e.frame);
          }
        }
      }
      
      /*
       * If all the dependencies to instantiate the joint in the simulator are met, the joints is instantated, otherwise the missing dependencies get tracked into the staticDependencies map
       */
      void EnvireJoints::itemAdded(const  envire::core::TypedItemAddedEvent<envire::core::Item<smurf::StaticTransformation>>& e){ 
        if (debug) { LOG_DEBUG( "[EnvireJoints::itemAdded] smurf::StaticTransformation received in Frame ***" + e.frame + "***");}
        smurf::StaticTransformation* smurfJoint = &(e.item->getData());
        // Upcast to a transformation, so we can use the same methods for dynamic joints
        //smurf::Transformation *pJoint = &smurfJoint;
        checkAndInstantiate<smurf::StaticTransformation*>(smurfJoint, e.frame);
      }
      
      void EnvireJoints::itemAdded(const TypedItemAddedEvent<Item<smurf::Joint>>& e){
        //LOG_DEBUG( "[Envire Joints] itemAdded: envire::core::Item<smurf::Joint>::Ptr>: " + e.frame);
        smurf::Joint* smurfJoint = &(e.item->getData());
        // Upcast to a transformation, so we can use the same methods for dynamic joints
        checkAndInstantiate<smurf::Joint*>(smurfJoint, e.frame);
      }
      
      
      // Private Methods
      // Static Joints
      /*
       * Checks if the there is a physical object created for the given frameName
       * 
       */
      bool EnvireJoints::getSimObject(const FrameId& frameName, std::shared_ptr<NodeInterface>& objectSim){
        bool found = false;
        using Iterator = EnvireGraph::ItemIterator<physicsNodeItem>;
        Iterator begin, end;
        boost::tie(begin, end) = control->graph->getItems<physicsNodeItem>(frameName);
        if (begin != end){
          objectSim = begin->getData();
          found = true;
          //LOG_DEBUG("[Envire Joints]::getSimObject: Found the physical object in frame "+ frameName);
        }
        return found;
      }
      
      /*
       * storageFrame is the frame in which the joint is stored to be recovered when the dependencies are met
       */
      void EnvireJoints::addDependencies(smurf::Transformation* smurfJoint, std::shared_ptr<mars::interfaces::NodeInterface>& sourceSim, std::shared_ptr<mars::interfaces::NodeInterface>& targetSim, FrameId storageFrame)
      {
        std::string dependencyName = smurfJoint->getSourceFrame().getName();
        if (! getSimObject(dependencyName, sourceSim))
        {
          dependencies[dependencyName].push_back(storageFrame); 
          //LOG_DEBUG("[Envire Joints] The joint " + smurfJoint->getName() + " misses the " + smurfJoint->getSourceFrame().getName() + " object. The dependency is now tracked. ");
        }
        dependencyName = smurfJoint->getTargetFrame().getName();
        if (! getSimObject(dependencyName, targetSim))
        {
          dependencies[dependencyName].push_back(storageFrame);
          //LOG_DEBUG("[Envire Joints] The joint " + smurfJoint->getName() + " misses the " + smurfJoint->getTargetFrame().getName() + " object. The dependency is now tracked. ");
        }
      }
      
      /*
       * See that the shared_ptrs of the source and target are in the graph
       * Set in the vector of missing physical objects these ones
       */
      bool EnvireJoints::instantiable(smurf::Transformation* smurfJoint, std::shared_ptr<mars::interfaces::NodeInterface>& sourceSim, std::shared_ptr<mars::interfaces::NodeInterface>& targetSim)
      {
        //LOG_DEBUG("[Envire Joints] The joint " + smurfJoint->getName() + " binds " + smurfJoint->getSourceFrame().getName() + " with " + smurfJoint->getTargetFrame().getName());
        bool instantiable = true;
        std::string dependencyName = smurfJoint->getSourceFrame().getName();
        if (! getSimObject(dependencyName, sourceSim))
        {
          instantiable = false;
        }
        dependencyName = smurfJoint->getTargetFrame().getName();
        if (! getSimObject(dependencyName, targetSim))
        {
          instantiable = false;
        }
        return instantiable;
      }
      
      void EnvireJoints::storeSimJoint (const std::shared_ptr<mars::interfaces::JointInterface>& jointInterface, mars::interfaces::JointData* jointData, FrameId storageFrame){

        physicsJointItemPtr physicsItem(new envire::core::Item<std::shared_ptr<mars::interfaces::JointInterface>>(jointInterface));
        // src/core/SimJoint.cpp:40:    SimJoint::SimJoint(ControlCenter *c, const JointData &sJoint_)
        // Create the simJoint from the control center and the jointData
        mars::sim::SimJoint* simJoint(new mars::sim::SimJoint(control, (*jointData)));
        simJoint->setInterface(jointInterface.get());
        // Make a shared_pointer directed to the simjoint and store the shared_ptr in the graph
        simJointItemPtr simJointItem(new envire::core::Item<std::shared_ptr<mars::sim::SimJoint>>(simJoint));
        control->graph->addItemToFrame(storageFrame, simJointItem);          
        control->graph->addItemToFrame(storageFrame, physicsItem);          
        //LOG_DEBUG("[Envire Joints] Joint interface stored after creation in frame: " +storageFrame);
        //
        //FIXME this guy should store a SimJoint and not a JointInterface
      }
      
      /*
       * Create the physical node data
       * 
       */
      void EnvireJoints::join(JointData* jointData, const std::shared_ptr<mars::interfaces::NodeInterface>& sourceSim, const std::shared_ptr<mars::interfaces::NodeInterface>& targetSim, FrameId storageFrame)
      {
        JointPhysics* jointPhysics(new JointPhysics(control->sim->getPhysics())); // Here maybe we should have used the PhysicsMapper --- I Don't know
        //JointInterface* jointInterface( new JointInterface(PhysicsMapper::newJointPhysics(control->sim->getPhysics())));
        std::shared_ptr<JointInterface> jointInterfacePtr(jointPhysics);
        //jointsPhysics.push_back(JointPhysics(*jointInterfacePtr));
        //JointInterface* jointInterface( new JointInterface(PhysicsMapper::newJointPhysics( control->sim->getPhysics())));
        //std::shared_ptr<mars::interfaces::JointInterface>jointInterfacePtr(jointInterface);
        //jointPhysics* jointPhysics( new PhysicsMapper::newJointPhysics(control->sim->getPhysics()))
        //LOG_DEBUG("[Envire Joints] Joint %d;  %d  ;  %d  ;  %.4f, %.4f, %.4f  ;  %.4f, %.4f, %.4f\n",  jointPhysics->index, jointPhysics->nodeIndex1, jointPhysics->nodeIndex2,  jointPhysics->anchor.x(), jointPhysics->anchor.y(), jointPhysics->anchor.z(), jointPhysics->axis1.x(), jointPhysics->axis1.y(), jointPhysics->axis1.z());
        if (debug){LOG_DEBUG("[Envire Joints] About to create joint " + jointData->name);}
        if(jointInterfacePtr->createJoint(jointData, sourceSim.get(), targetSim.get()))
        {
          if (debug) { LOG_DEBUG("[EnvireJoints::join] Physical joint '" + jointData->name + "' created.");}
          control->sim->sceneHasChanged(false);//important, otherwise the joint will be ignored by simulation
          storeSimJoint(jointInterfacePtr, jointData, storageFrame);
        }
        else
        {
          std::cerr << "ERROR: Failed to create joint" << std::endl;
          assert(false);
        }
      }
      
      /*
       * 
       * Static transformations are stored in the source frame.
       * 
       * For the static transformations there is sourceToTaget transformation but no AxisTransformation or ParentToJointOrigin.
       * 
       * TODO: Have the simulation center frame (in this case "center") as a constant for all the plugins.
       * 
       */
      void EnvireJoints::instantiate(smurf::StaticTransformation* smurfJoint, const std::shared_ptr<mars::interfaces::NodeInterface>& sourceSim, const std::shared_ptr<mars::interfaces::NodeInterface>& targetSim, FrameId storageFrame)
      {                                
        JointData* jointPhysics = new(JointData);
        std::string jointName = smurfJoint->getSourceFrame().getName() + "-" + smurfJoint->getTargetFrame().getName();
        jointPhysics->init(jointName, mars::interfaces::JOINT_TYPE_FIXED);
        envire::core::Transform jointPos = control->graph->getTransform("center", storageFrame);
        utils::Vector anchor = jointPos.transform.translation;
        jointPhysics->anchor = anchor;
        //LOG_DEBUG("[Envire Joints] Static Transformation. The vector anchor is: %.4f, %.4f, %.4f", anchor.x(), anchor.y(), anchor.z());
        join(jointPhysics, sourceSim, targetSim, storageFrame);
      }
      
      JointType EnvireJoints::getJointType(smurf::Joint* joint){
        boost::shared_ptr<urdf::Joint> jointModel = joint->getJointModel();
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
        if (debug) {LOG_DEBUG("[Envire Joints] The joint type is: " + logType); }
        return jointType;
      }
      
      /*
       * Sets anchor and axis1 of the jointPhysics
       * 
       * TODO: Anchor represents the point in space where the dynamic joints starts, right?
       * TODO: What represents axis1?
       * 
       * The axisTranformation seems to be always 0.0 0.0 0.0, this produces and error when creating the joint
       * 
       * TODO: Have the simulation center as a constant for all the envire plugins
       */
      void EnvireJoints::instantiate(smurf::Joint* smurfJoint, const std::shared_ptr<mars::interfaces::NodeInterface>& sourceSim, const std::shared_ptr<mars::interfaces::NodeInterface>& targetSim, FrameId storageFrame)
      {                                
        JointData* jointPhysics = new(JointData);
        std::string jointName = smurfJoint->getSourceFrame().getName() + "-" + smurfJoint->getTargetFrame().getName();        
        if (debug){ LOG_DEBUG("[Envire Joints] The jointName is " + jointName);}
        jointPhysics->init(jointName, getJointType(smurfJoint));
        envire::core::Transform jointPos = control->graph->getTransform("center", storageFrame);  // TODO What is this "center", the center of the robot? The simulation center?
        utils::Vector anchor = jointPos.transform.translation;
        jointPhysics->anchor = anchor;
        Eigen::Affine3d axisTf = smurfJoint -> getSourceToAxis();
        utils::Vector axis = axisTf.translation();
        jointPhysics->axis1 = axis;
        //jointPhysics->axis1.x() = 1.0; //Only work if I set here some value
        if (debug)
        {
            LOG_DEBUG("[Envire Joints]::Instantiate. The vector anchor is: %.4f, %.4f, %.4f", anchor.x(), anchor.y(), anchor.z());
            LOG_DEBUG("[Envire Joints]::Instantiate. The vector axis is: %.4f, %.4f, %.4f", axis.x(), axis.y(), axis.z());
        }
        join(jointPhysics, sourceSim, targetSim, storageFrame);
      }

      
      template <class jointType>
      void EnvireJoints::checkAndInstantiate(jointType smurfJoint, const FrameId storageFrame, bool addDeps)
      {
        std::shared_ptr<NodeInterface> sourceSim;
        std::shared_ptr<NodeInterface> targetSim;
        smurf::Transformation* smurfTf = smurfJoint;
        if (instantiable(smurfTf, sourceSim, targetSim))
        {
          instantiate(smurfJoint, sourceSim, targetSim, storageFrame);
        }
        else
        {
          if (addDeps)
            addDependencies(smurfTf, sourceSim, targetSim, storageFrame);
        }
      }
      
      
    } // end of namespace envire_joints
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_joints::EnvireJoints);
CREATE_LIB(mars::plugins::envire_joints::EnvireJoints);
