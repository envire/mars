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

namespace mars {
  namespace plugins {
    namespace envire_joints {
      
      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace envire::core;

      // Public Methods
      EnvireJoints::EnvireJoints(lib_manager::LibManager *theManager)
      : MarsPluginTemplate(theManager, "EnvireJoints"), 
      GraphEventDispatcher(){
      }
      
      void EnvireJoints::init() {
        assert(control->graph != nullptr);
        GraphEventDispatcher::subscribe(control->graph);
        GraphItemEventDispatcher<Item<smurf::StaticTransformation>::Ptr>::subscribe(control->graph);
        //GraphItemEventDispatcher<Item<smurf::Joint>::Ptr>::subscribe(control->graph);
        LOG_DEBUG("[Envire Joints] Init Method");
      }
      
      void EnvireJoints::reset() {
      }
      
      void EnvireJoints::update(sReal time_ms) {
      }
      
      void EnvireJoints::itemAdded(const  envire::core::TypedItemAddedEvent<envire::core::Item<smurf::StaticTransformation>::Ptr>& e){ 
        LOG_DEBUG( "[Envire Joints] itemAdded: envire::core::Item<smurf::StaticTransformation>::Ptr>");
        smurf::StaticTransformation smurfJoint = e.item->getData();
        std::shared_ptr<mars::interfaces::NodeInterface> sourceSim;
        std::shared_ptr<mars::interfaces::NodeInterface> targetSim;
        if (instantiable(smurfJoint, sourceSim, targetSim))
        {
          instantiate(smurfJoint, sourceSim, targetSim);
        }
      }
      
      // Private Methods
      // Static Joints
      /*
       * Checks if the there is a physical object created for the given frameName
       * 
       */
      bool EnvireJoints::getSimObject(const FrameId& frameName, std::shared_ptr<mars::interfaces::NodeInterface>& objectSim){
        bool found = false;
        LOG_DEBUG("[Envire Joints]::getSimObject: Frame name is "+ frameName);
        using physicsItemPtr = Item<std::shared_ptr<mars::interfaces::NodeInterface>>::Ptr;
        using Iterator = TransformGraph::ItemIterator<physicsItemPtr>;
        Iterator begin, end;
        boost::tie(begin, end) = control->graph->getItems<physicsItemPtr>(frameName);
        if (begin != end){
          objectSim = (*begin)->getData();
          found = true;
        }
        return found;
      }
      
      /*
       * See that the shared_ptrs of the source and target are in the graph
       * Set in the vector of missing physical objects these ones
       */
      bool EnvireJoints::instantiable(const smurf::StaticTransformation& smurfJoint, std::shared_ptr<mars::interfaces::NodeInterface>& sourceSim, std::shared_ptr<mars::interfaces::NodeInterface>& targetSim){
        LOG_DEBUG("[Envire Joints] The static joint " + smurfJoint.getName() + " binds " + smurfJoint.getSourceFrame().getName() + " with " + smurfJoint.getTargetFrame().getName());
        bool instantiable = true;
        std::string dependencieName = smurfJoint.getSourceFrame().getName();
        if (! getSimObject(dependencieName, sourceSim)) // This does not need a method
        {
          instantiable = false;
          staticDependencies[dependencieName].push_back(smurfJoint);
        }
        dependencieName = smurfJoint.getTargetFrame().getName();
        if (! getSimObject(dependencieName, targetSim))
        {
          instantiable = false;
          staticDependencies[dependencieName].push_back(smurfJoint);
        }
        return instantiable;
      }
      
      void EnvireJoints::storeSimJoint( const JointData& jointPhysics, std::shared_ptr<mars::interfaces::JointInterface>& jointInterface,                                        smurf::StaticTransformation smurfJoint){
        // TODO This method could be also for Dynamic Joints
        if (jointPhysics.type == mars::interfaces::JOINT_TYPE_FIXED){
          using physicsItemPtr = envire::core::Item<std::shared_ptr<mars::interfaces::JointInterface>>::Ptr;
          physicsItemPtr physicsItem(new envire::core::Item<std::shared_ptr<mars::interfaces::JointInterface>>(jointInterface));
          //FrameId storeFrame = smurfJoint.getSourceFrame().getName();
          control->graph->addItemToFrame(smurfJoint.getSourceFrame().getName(), physicsItem);          
        }
      }
      
      void EnvireJoints::join(JointData& jointPhysics, const std::shared_ptr<mars::interfaces::NodeInterface>& sourceSim, const std::shared_ptr<mars::interfaces::NodeInterface>& targetSim, const smurf::StaticTransformation smurfJoint){
        // TODO This methof could be for also dynamic joints
        std::shared_ptr<mars::interfaces::JointInterface> jointInterface(mars::sim::PhysicsMapper::newJointPhysics(control->sim->getPhysics()));
        // create the physical node data
        LOG_DEBUG("[Envire Joints] Just before creating the physical joint");
        if(jointInterface->createJoint(&jointPhysics, sourceSim.get(), targetSim.get()))
        {
          LOG_DEBUG("[Envire Joints] Just after creating the physical joint");
          control->sim->sceneHasChanged(false);//important, otherwise the joint will be ignored by simulation
          // TODO Store the share_ptr of the new joint in the correspondent frame, fixed joints get stored in the source frame, others have its own frame
          storeSimJoint(jointPhysics, jointInterface, smurfJoint);
        }
        else
        {
          std::cerr << "ERROR: Failed to create joint" << std::endl;
          assert(false);//this only happens if the JointConfigMapItem contains bad data, which it should never do
        }
      }
      
      void EnvireJoints::instantiate(const smurf::StaticTransformation smurfJoint, std::shared_ptr<mars::interfaces::NodeInterface>& sourceSim, std::shared_ptr<mars::interfaces::NodeInterface>& targetSim){                                
        // TODO This methof could be for also dynamic joints
        JointData jointPhysics; // We miss some data here
        std::string jointName = smurfJoint.getSourceFrame().getName() + "-" + smurfJoint.getTargetFrame().getName();
        jointPhysics.init(jointName, mars::interfaces::JOINT_TYPE_FIXED, 0, 0);
        join(jointPhysics, sourceSim, targetSim, smurfJoint);
      }
      
      /*
      void EnvireJoints::itemAdded(const TypedItemAddedEvent<mars::sim::JointConfigMapItem::Ptr>& e)
      {
        
        JointConfigMapItem::Ptr pItem = e.item;
        JointData jointData;
        if(jointData.fromConfigMap(&pItem->getData(), ""))
        {
          string id1 = pItem->getData().get<string>("itemId1", "");
          string id2 = pItem->getData().get<string>("itemId2", "");
          assert(!id1.empty());
          assert(!id2.empty());
          //the ids of the two items that should be connected by the joint
          boost::uuids::uuid uuid1 = boost::lexical_cast<boost::uuids::uuid>(id1);
          boost::uuids::uuid uuid2 = boost::lexical_cast<boost::uuids::uuid>(id2);
          join(jointData, uuid1, uuid2, e.item->getID());
        }
      }
      */

      

      
      /*
      void EnvireJoints::itemRemoved(const TypedItemRemovedEvent< mars::sim::JointConfigMapItem::Ptr >& e)
      {
        if(uuidToJoints.find(e.item->getID()) != uuidToJoints.end())
        {
          shared_ptr<JointInterface> jointInterface = uuidToJoints[e.item->getID()];
          //removing it from the map will destroy the interface because the map
          //was the only one that knew about the item.
          //This will also remove the joint from ode
          uuidToJoints.erase(e.item->getID());
        }
        else
        {
          std::cerr << "ERROR: Tried to remove a joint that was never added" << std::endl;
          assert(false);
        }
      }
      */
      

      /*
      std::shared_ptr<mars::interfaces::NodeInterface> EnvireJoints::getPhysicsInterface(const std::string& frameName){
        using physicsItemPtr = Item<std::shared_ptr<mars::interfaces::NodeInterface>>::Ptr;
        if (!(control->graph->containsItems<physicsItemPtr>(frameName)))
          LOG_DEBUG("[Envire Joints]::getPhysicsInterface: There is no share_ptr to a node interface in "+ frameName);
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
      */
      

      

        //For the dependencies a hash with the name been the frame and the contents the joints that depend on it
        // for each joint depending on it see if instantiable
      /*
      bool EnvireJoints::instantiable(const smurf::Joint& smurfJoint){
        bool instantiable = true;
        // See that the shared_ptrs of the source and target are in the graph
        // Set in the vector of missing physical objects these ones
        std::string jointName = smurfJoint.getName();
        // We now need the node id for the source and target object to bind, which corresponds to the shared_ptr of the simulated objects
        std::string sourceName = smurfJoint.getSourceFrame().getName();
        std::string targetName = smurfJoint.getTargetFrame().getName();
        LOG_DEBUG("[Envire Joints] The joint binds " + sourceName + " with " + targetName);
        if (! getSimObject(sourceName)) // This does not need a method
        {
          instantiable = false;
          dependencies[sourceName].push_back(smurfJoint);
        }
        if (! getSimObject(targetName))
        {
          instantiable = false;
          dependencies[targetName].push_back(smurfJoint);
        }
        return instantiable;
        */
        /*
        std::shared_ptr<NodeInterface> source = getPhysicsInterface(sourceName);
        std::shared_ptr<NodeInterface> target = getPhysicsInterface(targetName);
        */
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
      //}
      /*
      void EnvireJoints::addDependencies(std::vector<std::string> missingObjects, smurf::Joint dependentJoint){

      }
          
      void EnvireJoints::itemAdded(const TypedItemAddedEvent<Item<smurf::Joint>::Ptr>& e){
        smurf::Joint smurfJoint = e.item->getData();
        if (instantiable(smurfJoint)) //This also sets the dependencies if it isn't
        {
          instantiate(smurfJoint);
        }
      }
      
      void EnvireJoints::instantiate(smurf::Joint smurfJoint)
      {
        using namespace mars::interfaces;
        // The type of joints correspondences is taken from mars::smurf::SMURF::handleKinematics
        LOG_DEBUG("[Envire Joints] A joint of the graph will be instantiated");
        // See what type of joint is and generate the correspondent simulated one
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
        //control->graph->addItemToFrame(e.frame, physicsItem);
        //JointData marsJoint(jointName, jointType, uuidSource, uuidTarget);
        // We still need the Physicsuuid which is stored in the physics plugin...
      }
      */
      

      
    } // end of namespace envire_joints
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_joints::EnvireJoints);
CREATE_LIB(mars::plugins::envire_joints::EnvireJoints);
