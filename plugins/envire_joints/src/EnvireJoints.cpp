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




#include "EnvireJoints.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

#include <envire_core/graph/EnvireGraph.hpp>

#include <mars/sim/PhysicsMapper.h>

#include <urdf_model/model.h>
#include <mars/interfaces/sim/JointInterface.h>
#include <mars/interfaces/sim/NodeInterface.h>

// Comment-in the following line in order to get debug traces

namespace mars {
  namespace plugins {
    namespace envire_joints {
      
      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace envire::core;
      using namespace mars::sim;
      // TODO Are you sure that you need the shared_ptr in the Item?
      using NodeInterfacePtrItem = Item<std::shared_ptr<NodeInterface>>;
      using JointInterfacePtrItemPtr = Item<std::shared_ptr<mars::interfaces::JointInterface>>::Ptr;
      using SimJointPtrItemPtr = Item<std::shared_ptr<mars::sim::SimJoint>>::Ptr;
      using SimPtrItem = Item<std::shared_ptr<mars::sim::SimNode>>;

      // Public Methods
      EnvireJoints::EnvireJoints(lib_manager::LibManager *theManager): MarsPluginTemplate(theManager, "EnvireJoints"), GraphEventDispatcher(){
      }
      
      void EnvireJoints::init() {
        assert(control->graph != nullptr);
        GraphEventDispatcher::subscribe(control->graph.get());
        //GraphItemEventDispatcher<envire::core::Item<smurf::StaticTransformation>>::subscribe(control->graph.get());
        //GraphItemEventDispatcher<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>::subscribe(control->graph.get());
        //GraphItemEventDispatcher<envire::core::Item<smurf::Joint>>::subscribe(control->graph.get());
      }
      
      void EnvireJoints::reset() {
      }
      
      void EnvireJoints::update(sReal time_ms) {
      }
      
      void EnvireJoints::frameAdded(const FrameAddedEvent& e)
      {
          if(originId.empty())
          {
              originId = e.frame;
          }
      }
      
      void EnvireJoints::itemAdded(const  envire::core::TypedItemAddedEvent<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>& e){
#ifdef DEBUG
        LOG_DEBUG( "[EnvireJoints::itemAdded] mars::sim::SimNode received in Frame ***" + e.frame + "***");
#endif        
        using DynamicTfsIterator = EnvireGraph::ItemIterator<Item<smurf::Joint>>;
        using StaticTfsIterator = EnvireGraph::ItemIterator<Item<smurf::StaticTransformation>>;
        std::map<FrameId, std::vector<FrameId>>::iterator iterDeps = dependencies.find(e.frame);
        if (iterDeps != dependencies.end())
        {
#ifdef DEBUG
          LOG_DEBUG( "[EnvireJoints::itemAdded] Item added in frame '"+ e.frame + "' is matching at least a dependency"); 
#endif
          std::vector<FrameId> dependentFrames = dependencies[e.frame];
          for(FrameId frame : dependentFrames)
          {
            StaticTfsIterator beginStaticTfs, endStaticTfs;
            boost::tie(beginStaticTfs, endStaticTfs) = control->graph->getItems<Item<smurf::StaticTransformation>>(frame);
            if (beginStaticTfs != endStaticTfs)
            {
#ifdef DEBUG
              LOG_DEBUG( "[EnvireJoints::itemAdded] Item added in frame '"+ e.frame + "' is matching a static dependency"); 
#endif
              smurf::StaticTransformation* smurfTf = &(beginStaticTfs->getData());
              bool addDeps = false; // This avoids dependencies to be included twice
              checkAndInstantiate(smurfTf, addDeps);
            }
            else
            {
              // Dynamic transformations should have their own frame, therefore they can not be together in a frame with a static one
              DynamicTfsIterator beginDynTfs, endDynTfs;
              boost::tie(beginDynTfs, endDynTfs) = control->graph->getItems<Item<smurf::Joint>>(frame);
              if (beginDynTfs != endDynTfs)
              {
#ifdef DEBUG
                LOG_DEBUG( "[EnvireJoints::itemAdded] Item added in frame '"+ e.frame + "' is matching a dynamic dependency");
#endif
                smurf::Joint * smurfTf = &(beginDynTfs->getData());
                bool addDeps = false; // This avoids dependencies to be included twice
                checkAndInstantiate(smurfTf, addDeps);
              }
            }
            dependencies.erase(e.frame);
          }
        }
      }
      
      void EnvireJoints::itemAdded(const  envire::core::TypedItemAddedEvent<envire::core::Item<smurf::StaticTransformation>>& e){ 
#ifdef DEBUG
        LOG_DEBUG( "[EnvireJoints::itemAdded] smurf::StaticTransformation received in Frame ***" + e.frame + "***");
#endif
        smurf::StaticTransformation* smurfJoint = &(e.item->getData());
        checkAndInstantiate<smurf::StaticTransformation*>(smurfJoint);
      }
      
      void EnvireJoints::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Joint>>& e){
#ifdef DEBUG
        LOG_DEBUG( "[EnvireJoints::itemAdded] smurf::Joint received in Frame ***" + e.frame + "***");
#endif        
        smurf::Joint* smurfJoint = &(e.item->getData());
        createJointRecord(smurfJoint, e.frame);
        checkAndInstantiate<smurf::Joint*>(smurfJoint);
      }
      
      
      // Private Methods
      

      void EnvireJoints::createJointRecord(smurf::Joint *smurfJoint, const FrameId &frame)
      {
#ifdef DEBUG
        LOG_DEBUG( "[EnvireJoints::createJointRecord] ***" + frame + "***");
#endif        

        JointRecord* jointInfo(new JointRecord);
        jointInfo->name = smurfJoint->getName();
        Item<JointRecord>::Ptr jointItemPtr(new Item<JointRecord>(*jointInfo));
        control->graph->addItemToFrame(frame, jointItemPtr);          
      }

      template <class jointType>
      void EnvireJoints::checkAndInstantiate(jointType smurfJoint, bool addDeps)
      {
#ifdef DEBUG
        LOG_DEBUG( "[EnvireJoints::checkAndInstantiate]");
#endif      

        std::shared_ptr<mars::sim::SimNode> sourceSim;
        std::shared_ptr<mars::sim::SimNode> targetSim;
        smurf::Transformation* smurfTf = smurfJoint;
        if (instantiable(smurfTf, sourceSim, targetSim))
        {
          instantiate(smurfJoint, sourceSim, targetSim);
        }
        else
        {
          if (addDeps)
            addDependencies(smurfTf, sourceSim, targetSim);
        }
      }
      
      bool EnvireJoints::instantiable(smurf::Transformation* smurfJoint, std::shared_ptr<mars::sim::SimNode>& sourceSim, std::shared_ptr<mars::sim::SimNode>& targetSim)
      {
#ifdef DEBUG
            LOG_DEBUG("[EnvireJoints::instantiable] The joint " + smurfJoint->getName() + " binds " + smurfJoint->getSourceFrame().getName() + " with " + smurfJoint->getTargetFrame().getName());
#endif
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
      
      bool EnvireJoints::getSimObject(const FrameId& frameName, std::shared_ptr<mars::sim::SimNode>& objectSim){
#ifdef DEBUG
        LOG_DEBUG( "[EnvireJoints::getSimObject] ***" + frameName + "***");
#endif           
        bool found = false;
        using Iterator = EnvireGraph::ItemIterator<Item<std::shared_ptr<mars::sim::SimNode>>>;
        Iterator begin, end;
        boost::tie(begin, end) = control->graph->getItems<Item<std::shared_ptr<mars::sim::SimNode>>>(frameName);
        if (begin != end){
          objectSim = begin->getData();
          found = true;
#ifdef DEBUG
          LOG_DEBUG("[EnvireJoints::getSimObject] Found the physical object in frame "+ frameName);
#endif
        }
        return found;
      }
      
      template <class jointType>
      void EnvireJoints::instantiate(jointType smurfJoint, const std::shared_ptr<mars::sim::SimNode>& sourceSim, const std::shared_ptr<mars::sim::SimNode>& targetSim)
      {
#ifdef DEBUG
          LOG_DEBUG("[EnvireJoints::instantiate] The joint name is: %s", smurfJoint->getName().c_str());
#endif
        JointData* jointData = new(JointData);
        jointData->init(smurfJoint->getName(), getJointType(smurfJoint));
        setAxis1(smurfJoint, jointData);
        setAnchor(smurfJoint, jointData);
        setLimits(smurfJoint, jointData);
        std::shared_ptr<JointInterface> jointInterfacePtr = join(jointData, sourceSim, targetSim);
#ifdef DEBUG
          LOG_DEBUG("[EnvireJoints::instantiate] The storageFrame is: %s", smurfJoint->getTargetFrame().getName().c_str());
#endif
        storeSimJoint(jointInterfacePtr, jointData, smurfJoint->getSourceFrame().getName());
      }

      void EnvireJoints::setLimits(smurf::Joint* smurfJoint, mars::interfaces::JointData* jointData)
      {
        std::pair<double, double> position_limits = smurfJoint->getPositionLimits();
        jointData->lowStopAxis1 = position_limits.first;
        jointData->highStopAxis1 = position_limits.second;

        // DIRTY FIX!!! 
        jointData->damping_const_constraint_axis1 = 1.0;
        jointData->spring_const_constraint_axis1 = 10000.0;
        jointData->damping_constant = 1.0;
        jointData->spring_constant = 10000.0;

        //jointData->print();
      }

      void EnvireJoints::setLimits(smurf::StaticTransformation* smurfJoint, mars::interfaces::JointData* jointData)
      {}
      
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
#ifdef DEBUG
        LOG_DEBUG("[Envire Joints] The joint type is: " + logType); 
#endif
        return jointType;
      }

      void EnvireJoints::setAxis1(smurf::StaticTransformation* smurfJoint, JointData* jointData){ }
      
      void EnvireJoints::setAxis1(smurf::Joint* smurfJoint, JointData* jointData){ 
        envire::core::FrameId targetFrame = smurfJoint->getTargetFrame().getName();
        envire::core::Transform targetPos = control->graph->getTransform(originId, targetFrame); 
        Eigen::Affine3d axisTf = targetPos.transform.orientation * smurfJoint -> getSourceToAxis().inverse();
        jointData->axis1 = axisTf.translation();
#ifdef DEBUG
        LOG_DEBUG("[EnvireJoints::instantiate] The vector axis is: %.4f, %.4f, %.4f", jointData->axis1.x(), jointData->axis1.y(), jointData->axis1.z());
#endif
      }      
    
      template <class jointType>
      void EnvireJoints::setAnchor(jointType* smurfJoint, JointData* jointData){
        envire::core::FrameId targetFrame = smurfJoint->getTargetFrame().getName();
        envire::core::Transform targetPos = control->graph->getTransform(originId, targetFrame); 
        utils::Vector anchor = targetPos.transform.translation;
        jointData->anchor = anchor;
#ifdef DEBUG
        LOG_DEBUG("[EnvireJoints::instantiate] The vector anchor is: %.4f, %.4f, %.4f", jointData->anchor.x(), jointData->anchor.y(), jointData->anchor.z());
#endif
      }
      
      std::shared_ptr<JointInterface> EnvireJoints::join(JointData* jointData, const std::shared_ptr<mars::sim::SimNode>& sourceSim, const std::shared_ptr<mars::sim::SimNode>& targetSim)
      {
        JointPhysics* jointPhysics(new JointPhysics(control->sim->getPhysics())); 
        std::shared_ptr<JointInterface> jointInterfacePtr(jointPhysics);
        if(jointInterfacePtr->createJoint(jointData, sourceSim->getInterface(), targetSim->getInterface()))
        {
#ifdef DEBUG
          LOG_DEBUG("[EnvireJoints::join] Physical joint '" + jointData->name + "' created.");
#endif
          control->sim->sceneHasChanged(false);//important, otherwise the joint will be ignored by simulation

        }
        else
        {
          std::cerr << "ERROR: Failed to create joint" << std::endl;
          assert(false);
        }
        return jointInterfacePtr;
      }
      
      void EnvireJoints::storeSimJoint (const std::shared_ptr<mars::interfaces::JointInterface>& jointInterface, mars::interfaces::JointData* jointData, FrameId storageFrame){
        JointInterfacePtrItemPtr jointInterfacePtrItemPtr(new envire::core::Item<std::shared_ptr<mars::interfaces::JointInterface>>(jointInterface));
        // src/core/SimJoint.cpp:40:    SimJoint::SimJoint(ControlCenter *c, const JointData &sJoint_)
        std::shared_ptr<mars::sim::SimJoint> simJoint(new mars::sim::SimJoint(control, (*jointData)));
        simJoint->setPhysicalJoint(jointInterface.get());
        SimJointPtrItemPtr simJointPtrItemPtr(new envire::core::Item<std::shared_ptr<mars::sim::SimJoint>>(simJoint));
        control->graph->addItemToFrame(storageFrame, simJointPtrItemPtr);          
        control->graph->addItemToFrame(storageFrame, jointInterfacePtrItemPtr);          
        addToJointRecord(storageFrame, jointData->name, simJoint.get(), jointInterface.get());
      }

      void EnvireJoints::addToJointRecord(const envire::core::FrameId &frameId, const std::string &jointName, mars::sim::SimJoint *simJoint, mars::interfaces::JointInterface *jointInterface)
      {
        using RecordIterator = EnvireGraph::ItemIterator<Item<JointRecord>>;
        RecordIterator begin, end;
        // TODO Ask Arne, why to getItems I have to provide the template Item?
        // won't getItems only look for Items?
        boost::tie(begin, end) = control->graph->getItems<Item<JointRecord>>(frameId);
        bool stored=false;
        while((!stored)&&(begin!=end))
        {
          JointRecord* record = &(begin->getData());
          if (record->name == jointName)
          {
            record->sim = std::shared_ptr<mars::sim::SimJoint>(simJoint);
            record->interface = std::shared_ptr<mars::interfaces::JointInterface>(jointInterface);
          }
          begin ++;
        }

      };


      void EnvireJoints::addDependencies(smurf::Transformation* smurfJoint, std::shared_ptr<mars::sim::SimNode>& sourceSim, std::shared_ptr<mars::sim::SimNode>& targetSim)
      {
        std::string dependencyName = smurfJoint->getSourceFrame().getName();
        if (! getSimObject(dependencyName, sourceSim))
        {
          dependencies[dependencyName].push_back(smurfJoint->getSourceFrame().getName()); 
          //LOG_DEBUG("[Envire Joints] The joint " + smurfJoint->getName() + " misses the " + smurfJoint->getSourceFrame().getName() + " object. The dependency is now tracked. ");
        }
        dependencyName = smurfJoint->getTargetFrame().getName();
        if (! getSimObject(dependencyName, targetSim))
        {
          dependencies[dependencyName].push_back(smurfJoint->getTargetFrame().getName());
          //LOG_DEBUG("[Envire Joints] The joint " + smurfJoint->getName() + " misses the " + smurfJoint->getTargetFrame().getName() + " object. The dependency is now tracked. ");
        }
      }
    } // end of namespace envire_joints
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_joints::EnvireJoints);
CREATE_LIB(mars::plugins::envire_joints::EnvireJoints);
