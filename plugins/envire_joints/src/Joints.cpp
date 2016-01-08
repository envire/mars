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
#include <mars/sim/PhysicsMapper.h>

#include <mars/interfaces/sim/JointInterface.h>
#include <mars/interfaces/sim/NodeInterface.h>

namespace mars {
  namespace plugins {
    namespace envire_joints {
      
      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace envire::core;
      using physicsNodeItemPtr = Item<std::shared_ptr<NodeInterface>>::Ptr;
      using physicsJointItemPtr = envire::core::Item<std::shared_ptr<mars::interfaces::JointInterface>>::Ptr;

      // Public Methods
      EnvireJoints::EnvireJoints(lib_manager::LibManager *theManager): MarsPluginTemplate(theManager, "EnvireJoints"), GraphEventDispatcher(){
      }
      
      void EnvireJoints::init() {
        assert(control->graph != nullptr);
        GraphEventDispatcher::subscribe(control->graph);
        GraphItemEventDispatcher<Item<smurf::StaticTransformation>::Ptr>::subscribe(control->graph);
        GraphItemEventDispatcher<Item<std::shared_ptr<NodeInterface>>::Ptr>::subscribe(control->graph);
        GraphItemEventDispatcher<Item<smurf::Joint>::Ptr>::subscribe(control->graph);
        //LOG_DEBUG("[Envire Joints] Init Method");
      }
      
      void EnvireJoints::reset() {
      }
      
      void EnvireJoints::update(sReal time_ms) {
      }
      


      /*
       * When a new physical object is added it is checked if there is any joint missing that physical object to be generated. This is checked using the staticDependencies map
       */
      void EnvireJoints::itemAdded(const  envire::core::TypedItemAddedEvent<envire::core::Item<std::shared_ptr<NodeInterface>>::Ptr>& e){
        using staticTfsIterator = TransformGraph::ItemIterator<Item<smurf::StaticTransformation>::Ptr>;
        using dynamicTfsIterator = TransformGraph::ItemIterator<Item<smurf::Joint>::Ptr>;
        //LOG_DEBUG( "[Envire Joints] itemAdded: envire::core::Item<std::shared_ptr<NodeInterface>>::Ptr: in frame "+ e.frame);
        std::map<FrameId, std::vector<FrameId>>::iterator iterStatic = dependencies.find(e.frame);
        if (iterStatic != dependencies.end()){
          //LOG_DEBUG( "[Envire Joints] itemAdded in frame "+ e.frame + " is matching a dependency");
          std::vector<FrameId> dependentFrames = dependencies[e.frame];
          for(FrameId frame : dependentFrames){
            staticTfsIterator beginTfs, endTfs;
            boost::tie(beginTfs, endTfs) = control->graph->getItems<Item<smurf::StaticTransformation>::Ptr>(frame);
            if (beginTfs != endTfs)
            {
              smurf::StaticTransformation* smurfTf = &((*beginTfs)->getData());
              bool addDeps = false; // So that no dependencies are included twice
              checkAndInstantiate(smurfTf, frame, addDeps);
            }
            else
            {
              dynamicTfsIterator beginTfs, endTfs;
              boost::tie(beginTfs, endTfs) = control->graph->getItems<Item<smurf::Joint>::Ptr>(frame);
              if (beginTfs != endTfs)
              {
                smurf::Joint * smurfTf = &((*beginTfs)->getData());
                bool addDeps = false;
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
      void EnvireJoints::itemAdded(const  envire::core::TypedItemAddedEvent<envire::core::Item<smurf::StaticTransformation>::Ptr>& e){ 
        //LOG_DEBUG( "[Envire Joints] itemAdded: envire::core::Item<smurf::StaticTransformation>::Ptr>" + e.frame);
        smurf::StaticTransformation* smurfJoint = &(e.item->getData());
        // Upcast to a transformation, so we can use the same methods for dynamic joints
        //smurf::Transformation *pJoint = &smurfJoint;
        checkAndInstantiate<smurf::StaticTransformation*>(smurfJoint, e.frame);
      }
      
      void EnvireJoints::itemAdded(const TypedItemAddedEvent<Item<smurf::Joint>::Ptr>& e){
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
        using Iterator = TransformGraph::ItemIterator<physicsNodeItemPtr>;
        Iterator begin, end;
        boost::tie(begin, end) = control->graph->getItems<physicsNodeItemPtr>(frameName);
        if (begin != end){
          objectSim = (*begin)->getData();
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
      
      void EnvireJoints::storeSimJoint(std::shared_ptr<mars::interfaces::JointInterface>& jointInterface, FrameId storageFrame){

        physicsJointItemPtr physicsItem(new envire::core::Item<std::shared_ptr<mars::interfaces::JointInterface>>(jointInterface));
        control->graph->addItemToFrame(storageFrame, physicsItem);          
        //LOG_DEBUG("[Envire Joints] Joint interface stored after creation in frame: " +storageFrame);
      }
      
      /*
       * Create the physical node data
       * 
       */
      void EnvireJoints::join(JointData* jointPhysics, const std::shared_ptr<mars::interfaces::NodeInterface>& sourceSim, const std::shared_ptr<mars::interfaces::NodeInterface>& targetSim, FrameId storageFrame)
      {
        std::shared_ptr<mars::interfaces::JointInterface> jointInterface(mars::sim::PhysicsMapper::newJointPhysics(control->sim->getPhysics()));
        //LOG_DEBUG("[Envire Joints] Joint %d;  %d  ;  %d  ;  %.4f, %.4f, %.4f  ;  %.4f, %.4f, %.4f\n",  jointPhysics->index, jointPhysics->nodeIndex1, jointPhysics->nodeIndex2,  jointPhysics->anchor.x(), jointPhysics->anchor.y(), jointPhysics->anchor.z(), jointPhysics->axis1.x(), jointPhysics->axis1.y(), jointPhysics->axis1.z());
        //LOG_DEBUG("[Envire Joints] About to create joint " + jointPhysics->name);
        if(jointInterface->createJoint(jointPhysics, sourceSim.get(), targetSim.get()))
        {
          //LOG_DEBUG("[Envire Joints] Objects joined");
          control->sim->sceneHasChanged(false);//important, otherwise the joint will be ignored by simulation
          storeSimJoint(jointInterface, storageFrame);
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
        envire::core::Transform jointPos = control->graph->getTransform("center", storageFrame); // 
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
