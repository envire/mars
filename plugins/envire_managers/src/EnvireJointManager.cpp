/*
 *  Copyright 2011, 2012, DFKI GmbH Robotics Innovation Center
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
 * \file EnvireJointManager.cpp
 * \author Malte Roemmermann
 * \brief "EnvireJointManager" is the class that manage all joints and their
 * operations and communication between the different modules of the simulation.
 *
 */

#include "EnvireJointManager.hpp"

#include <mars/sim/SimNode.h>
#include <mars/sim/SimJoint.h>
#include <mars/sim/PhysicsMapper.h>


#include <stdexcept>

#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/utils/mathUtils.h>
#include <mars/utils/MutexLocker.h>
#include <mars/interfaces/Logging.hpp>
#include <mars/data_broker/DataBrokerInterface.h>

namespace mars {
  namespace plugins {
    namespace envire_managers{

    /**
     *\brief Initialization of a new EnvireJointManager
     *
     * pre:
     *     - a pointer to a ControlCenter is needed
     * post:
     *     - next_node_id should be initialized to one
     */
    EnvireJointManager::EnvireJointManager(mars::interfaces::ControlCenter *c) {
      control = c;
      next_joint_id = 1;
    }

    unsigned long EnvireJointManager::addJoint(mars::interfaces::JointData *jointS, bool reload) {

      if (jointS->frameID.empty())
            jointS->frameID = jointS->name;

      LOG_DEBUG("EnvireJointManager::addJoint: " + jointS->name);

      std::shared_ptr<mars::interfaces::JointInterface> newJointInterface;
      std::vector<mars::sim::SimNode*>::iterator iter;
      
      
      mars::interfaces::NodeInterface *i_node1 = 0;
      mars::interfaces::NodeInterface *i_node2 = 0;
      mars::utils::Vector an;

      if (!reload) {
        LOG_ERROR("EnvireJointManager::addNode: Reload is not implemented: " + jointS->name);
      //  iMutex.lock();
      //  simJointsReload.push_back(*jointS);
      //  iMutex.unlock();
      }

      //if(jointS->axis1.lengthSquared() < Vector::EPSILON && jointS->type != JOINT_TYPE_FIXED) {
      if(jointS->axis1.squaredNorm() < mars::utils::EPSILON && jointS->type != mars::interfaces::JOINT_TYPE_FIXED) {
        LOG_ERROR("EnvireJointManager::Cannot create joint without axis1 " + jointS->name);
        return 0;
      }

      // create an interface object to the physics
      newJointInterface.reset(mars::sim::PhysicsMapper::newJointPhysics(control->sim->getPhysics()));
      // reset the anchor
      //if node index is 0, the node connects to the environment.
      std::shared_ptr<mars::sim::SimNode> node1 = control->nodes->getSimNode(jointS->nodeIndex1);
      LOG_DEBUG("EnvireJointManager::addJoint: node1: " + node1->getName());
      if (node1) 
        i_node1 = node1->getInterface();

      std::shared_ptr<mars::sim::SimNode> node2 = control->nodes->getSimNode(jointS->nodeIndex2);

      LOG_DEBUG("EnvireJointManager::addJoint: node2: " + node2->getName());

      if (node2) i_node2 = node2->getInterface();

      // ### important! how to deal with different load options? ###
      //if (load_option == OPEN_INITIAL)
      //jointS->angle1_offset = jointS->angle2_offset = 0;
      // TODO: set anchor!
      if (jointS->anchorPos == mars::interfaces::ANCHOR_NODE1) {
        assert(node1);
        jointS->anchor = node1->getPosition();
      } else if (jointS->anchorPos == mars::interfaces::ANCHOR_NODE2) {
        assert(node2);
        jointS->anchor = node2->getPosition();
      } else if (jointS->anchorPos == mars::interfaces::ANCHOR_CENTER) {
        assert(node1);
        assert(node2);
        jointS->anchor = (node1->getPosition() + node2->getPosition()) / 2.;
      }

      // create the physical node data
      if (newJointInterface->createJoint(jointS, i_node1, i_node2)) {
        // put all data to the correct place
        iMutex.lock();
        // set the next free id
        jointS->index = next_joint_id;
        next_joint_id++;
        std::shared_ptr<mars::sim::SimJoint> newJoint(new mars::sim::SimJoint(control, *jointS));
        newJoint->setAttachedNodes(node1, node2);
        //    newJoint->setSJoint(*jointS);
        newJoint->setPhysicalJoint(newJointInterface);

        SimJointItemPtr newJointItemPtr( new SimJointItem(newJoint));        
        control->graph->addItemToFrame(jointS->frameID, newJointItemPtr);   

        simJoints[jointS->index] = newJointItemPtr;
        iMutex.unlock();
        control->sim->sceneHasChanged(false);
        return jointS->index;
      } else {
        std::cerr << "EnvireJointManager: Could not create new joint (JointInterface::createJoint() returned false)." << std::endl;
        // if no node was created in physics
        // delete the objects
        newJointInterface.reset();
        // and return false
        return 0;
      }
    }

    int EnvireJointManager::getJointCount() {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      //MutexLocker locker(&iMutex);
      //return simJoints.size();
    }

    void EnvireJointManager::editJoint(mars::interfaces::JointData *jointS) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // MutexLocker locker(&iMutex);
      // std::map<unsigned long, SimJoint*>::iterator iter = simJoints.find(jointS->index);
      // if (iter != simJoints.end()) {
      //   iter->second->setAnchor(jointS->anchor);
      //   iter->second->setAxis(jointS->axis1);
      //   iter->second->setAxis(jointS->axis2, 2);
      //   iter->second->setLowerLimit(jointS->lowStopAxis1);
      //   iter->second->setUpperLimit(jointS->highStopAxis1);
      //   iter->second->setLowerLimit(jointS->lowStopAxis2, 2);
      //   iter->second->setUpperLimit(jointS->highStopAxis2, 2);
      // }
    }

    void EnvireJointManager::getListJoints(std::vector<mars::interfaces::core_objects_exchange>* jointList) {
      mars::interfaces::core_objects_exchange obj;
      JointMap::iterator iter;
      mars::utils::MutexLocker locker(&iMutex);
      jointList->clear();
      for (iter = simJoints.begin(); iter != simJoints.end(); iter++) {
        iter->second->getData()->getCoreExchange(&obj);
        jointList->push_back(obj);
      }
    }

    void EnvireJointManager::getJointExchange(unsigned long id,
                                        mars::interfaces::core_objects_exchange* obj) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // MutexLocker locker(&iMutex);
      // std::map<unsigned long, SimJoint*>::iterator iter = simJoints.find(id);
      // if (iter != simJoints.end())
      //   iter->second->getCoreExchange(obj);
      // else
      //   obj = NULL;
    }


    const mars::interfaces::JointData EnvireJointManager::getFullJoint(unsigned long index) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // MutexLocker locker(&iMutex);
      // map<unsigned long, SimJoint*>::iterator iter = simJoints.find(index);
      // if (iter != simJoints.end())
      //   return iter->second->getSJoint();
      // else {
      //   char msg[128];
      //   sprintf(msg, "could not find joint with index: %lu", index);
      //   throw std::runtime_error(msg);
      // }
    }

    void EnvireJointManager::removeJoint(unsigned long index) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // SimJoint* tmpJoint = 0;
      // MutexLocker locker(&iMutex);
      // map<unsigned long, SimJoint*>::iterator iter = simJoints.find(index);

      // if (iter != simJoints.end()) {
      //   tmpJoint = iter->second;
      //   simJoints.erase(iter);
      // }

      // control->motors->removeJointFromMotors(index);

      // if (tmpJoint)
      //   delete tmpJoint;
      // control->sim->sceneHasChanged(false);
    }

    void EnvireJointManager::removeJointByIDs(unsigned long id1, unsigned long id2) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      //map<unsigned long, SimJoint*>::iterator iter;
      // iMutex.lock();

      // for (iter = simJoints.begin(); iter != simJoints.end(); iter++)
      //   if((iter->second->getNodeId() == id1 &&
      //       iter->second->getNodeId(2) == id2) ||
      //      (iter->second->getNodeId() == id2 &&
      //       iter->second->getNodeId(2) == id1)) {
      //     iMutex.unlock();
      //     removeJoint(iter->first);
      //     return;
      //   }
      // iMutex.unlock();
    }

    std::shared_ptr<mars::sim::SimJoint> EnvireJointManager::getSimJoint(unsigned long id){
      mars::utils::MutexLocker locker(&iMutex);
      JointMap::iterator iter = simJoints.find(id);
      if (iter != simJoints.end())
        return iter->second->getData();
      else
        return std::shared_ptr<mars::sim::SimJoint>();
    }


    std::vector<mars::sim::SimJoint*> EnvireJointManager::getSimJoints(void) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // vector<SimJoint*> v_simJoints;
      // map<unsigned long, SimJoint*>::iterator iter;
      // MutexLocker locker(&iMutex);
      // for (iter = simJoints.begin(); iter != simJoints.end(); iter++)
      //   v_simJoints.push_back(iter->second);
      // return v_simJoints;
    }


    void EnvireJointManager::reattacheJoints(unsigned long node_id) {
       printf("not implemented : %s\n", __PRETTY_FUNCTION__);
//       JointMap::iterator iter;
//       MutexLocker locker(&iMutex);
//       for (iter = simJoints.begin(); iter != simJoints.end(); iter++) {
//         if (iter->second->getSJoint().nodeIndex1 == node_id ||
//             iter->second->getSJoint().nodeIndex2 == node_id) {
//           iter->second->reattachJoint();
//         }
//      }
    }

    void EnvireJointManager::reloadJoints(void) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // list<JointData>::iterator iter;
      // //MutexLocker locker(&iMutex);
      // for(iter = simJointsReload.begin(); iter != simJointsReload.end(); iter++)
      //   addJoint(&(*iter), true);
    }

    void EnvireJointManager::updateJoints(mars::interfaces::sReal calc_ms) {
      mars::utils::MutexLocker locker(&iMutex);
      JointMap::iterator iter;
      for(iter = simJoints.begin(); iter != simJoints.end(); iter++) {
        iter->second->getData()->update(calc_ms);
      }
    }


    void EnvireJointManager::updatePositionsFromGraph(){

        mars::utils::MutexLocker locker(&iMutex);

        for (JointMap::iterator iter = simJoints.begin(); iter != simJoints.end(); iter++) {
            const std::shared_ptr<mars::sim::SimJoint> sim_joint = iter->second->getData();

            sim_joint->reattachJoint();

        }
    }



    void EnvireJointManager::clearAllJoints(bool clear_all) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::iterator iter;
      // MutexLocker locker(&iMutex);
      // if(clear_all) simJointsReload.clear();

      // while(!simJoints.empty()) {
      //   control->motors->removeJointFromMotors(simJoints.begin()->first);
      //   delete simJoints.begin()->second;
      //   simJoints.erase(simJoints.begin());
      // }
      // control->sim->sceneHasChanged(false);

      // next_joint_id = 1;
    }

    std::list<mars::interfaces::JointData>::iterator EnvireJointManager::getReloadJoint(unsigned long id) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // std::list<JointData>::iterator iter = simJointsReload.begin();
      // for(;iter!=simJointsReload.end(); ++iter) {
      //   if(iter->index == id) break;
      // }
      // return iter;
    }

    void EnvireJointManager::setReloadJointOffset(unsigned long id, mars::interfaces::sReal offset) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // MutexLocker locker(&iMutex);
      // list<JointData>::iterator iter = getReloadJoint(id);
      // if (iter != simJointsReload.end())
      //   iter->angle1_offset = offset;
    }

    void EnvireJointManager::setReloadJointAxis(unsigned long id, const mars::utils::Vector &axis) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // MutexLocker locker(&iMutex);
      // list<JointData>::iterator iter = getReloadJoint(id);
      // if (iter != simJointsReload.end())
      //   iter->axis1 = axis;
    }


    void EnvireJointManager::scaleReloadJoints(mars::interfaces::sReal x_factor, mars::interfaces::sReal y_factor, mars::interfaces::sReal z_factor)
    {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // list<JointData>::iterator iter;
      // MutexLocker locker(&iMutex);
      // for(iter = simJointsReload.begin(); iter != simJointsReload.end(); iter++) {
      //   iter->anchor.x() *= x_factor;
      //   iter->anchor.y() *= y_factor;
      //   iter->anchor.z() *= z_factor;
      // }
    }


    void EnvireJointManager::setJointTorque(unsigned long id, mars::interfaces::sReal torque) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // MutexLocker locker(&iMutex);
      // map<unsigned long, SimJoint*>::iterator iter = simJoints.find(id);
      // if (iter != simJoints.end())
      //   iter->second->setEffort(torque, 0);
    }


    void EnvireJointManager::changeStepSize(void) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::iterator iter;
      // MutexLocker locker(&iMutex);
      // for (iter = simJoints.begin(); iter != simJoints.end(); iter++) {
      //   iter->second->updateStepSize();
      // }
    }

    void EnvireJointManager::setReloadAnchor(unsigned long id, const mars::utils::Vector &anchor) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // MutexLocker locker(&iMutex);
      // list<JointData>::iterator iter = getReloadJoint(id);
      // if (iter != simJointsReload.end())
      //   iter->anchor = anchor;
    }


    void EnvireJointManager::setSDParams(unsigned long id, mars::interfaces::JointData *sJoint) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // MutexLocker locker(&iMutex);
      // map<unsigned long, SimJoint*>::iterator iter = simJoints.find(id);
      // if (iter != simJoints.end())
      //   iter->second->setSDParams(sJoint);
    }


    void EnvireJointManager::setVelocity(unsigned long id, mars::interfaces::sReal velocity) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // MutexLocker locker(&iMutex);
      // map<unsigned long, SimJoint*>::iterator iter = simJoints.find(id);
      // if (iter != simJoints.end())
      //   iter->second->setVelocity(velocity);
    }


    void EnvireJointManager::setVelocity2(unsigned long id, mars::interfaces::sReal velocity) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // MutexLocker locker(&iMutex);
      // map<unsigned long, SimJoint*>::iterator iter = simJoints.find(id);
      // if (iter != simJoints.end())
      //   iter->second->setVelocity(velocity, 2);
    }


    void EnvireJointManager::setForceLimit(unsigned long id, mars::interfaces::sReal max_force,
                                     bool first_axis) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // MutexLocker locker(&iMutex);
      // map<unsigned long, SimJoint*>::iterator iter = simJoints.find(id);
      // if (iter != simJoints.end()) {
      //   if (first_axis)
      //     iter->second->setEffortLimit(max_force);
      //   else
      //     iter->second->setEffortLimit(max_force, 2);
      // }
    }

    unsigned long EnvireJointManager::getID(const std::string& joint_name) const {
      JointMap::const_iterator iter;
      mars::utils::MutexLocker locker(&iMutex);
      for(iter = simJoints.begin(); iter != simJoints.end(); iter++) {
        mars::interfaces::JointData joint = iter->second->getData()->getSJoint();
        if (joint.name == joint_name)
          return joint.index;
      }
      return 0;
    }

    bool EnvireJointManager::getDataBrokerNames(unsigned long id, std::string *groupName,
                                          std::string *dataName) const {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return false;
      // iter->second->getDataBrokerNames(groupName, dataName);
      // return true;
    }

    void EnvireJointManager::setOfflineValue(unsigned long id, mars::interfaces::sReal value) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return;
      // iter->second->setOfflinePosition(value);
    }

    mars::interfaces::sReal EnvireJointManager::getLowStop(unsigned long id) const {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return 0.;
      // return iter->second->getLowerLimit();
    }

    mars::interfaces::sReal EnvireJointManager::getHighStop(unsigned long id) const {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return 0.;
      // return iter->second->getUpperLimit();
    }

    mars::interfaces::sReal EnvireJointManager::getLowStop2(unsigned long id) const {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return 0.;
      // return iter->second->getLowerLimit(2);
    }

    mars::interfaces::sReal EnvireJointManager::getHighStop2(unsigned long id) const {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return 0.;
      // return iter->second->getUpperLimit(2);
    }

    void EnvireJointManager::setLowStop(unsigned long id, mars::interfaces::sReal lowStop) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return;
      // return iter->second->setLowerLimit(lowStop);
    }

    void EnvireJointManager::setHighStop(unsigned long id, mars::interfaces::sReal highStop) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return;
      // return iter->second->setUpperLimit(highStop);
    }

    void EnvireJointManager::setLowStop2(unsigned long id, mars::interfaces::sReal lowStop2) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return;
      // return iter->second->setLowerLimit(lowStop2, 2);
    }

    void EnvireJointManager::setHighStop2(unsigned long id, mars::interfaces::sReal highStop2) {
      printf("not implemented : %s\n", __PRETTY_FUNCTION__);
      // map<unsigned long, SimJoint*>::const_iterator iter;
      // iter = simJoints.find(id);
      // if(iter == simJoints.end())
      //   return;
      // return iter->second->setUpperLimit(highStop2, 2);
    }

  }
  } // end of namespace sim
} // end of namespace mars
