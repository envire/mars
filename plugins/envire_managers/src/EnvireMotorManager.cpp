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
 * \file EnvireMotorManager.cpp
 * \author  Vladimir Komsiyski
 * \brief "EnvireMotorManager" implements EnvireMotorManagerInterface.
 * It is manages all motors and all motor 
 * operations that are used for the communication between the simulation 
 * modules.
 *
 * \version 1.3
 * Moved from the Simulator class
 * \date 07.07.2011
 */

#include "EnvireMotorManager.hpp"

#include <mars/sim/SimNode.h>
#include <mars/sim/SimMotor.h>
#include <mars/sim/SimJoint.h>
#include <mars/sim/JointRecord.h>
#include <mars/sim/PhysicsMapper.h>



#include <stdexcept>

#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/JointManagerInterface.h>
#include <mars/utils/MutexLocker.h>
#include <mars/interfaces/Logging.hpp>
#include <mars/utils/mathUtils.h>

#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>



namespace mars {
  namespace plugins {
  namespace envire_managers {
  
    using namespace std;
    using namespace utils;
    using namespace interfaces;

    /**
     * \brief Constructor.
     *
     * \param c The pointer to the ControlCenter of the simulation.
     */ 
    EnvireMotorManager::EnvireMotorManager(ControlCenter *c)
    {
      control = c;
      next_motor_id = 1;
    }

    /**
     * \brief Add a motor to the simulation.
     *
     * \param motorS A pointer to the MotorData that defines the new motor.
     *
     * \param reload Used internally by the simulation. The
     * default value is \c false. If this param is set to \c true the new motor
     * will not be reloaded after a reset of the simulation.
     *
     * \return The unique id of the newly added motor.
     */

    unsigned long EnvireMotorManager::addMotor(MotorData *motorS, bool reload) 
    {  
      iMutex.lock();
      motorS->index = next_motor_id;
      next_motor_id++;
      iMutex.unlock();
      if (!reload) {
        iMutex.lock();
        simMotorsReload.push_back(*motorS);
        iMutex.unlock();
      }
      mars::sim::SimMotor* simMotor = new mars::sim::SimMotor(control, *motorS);
      std::shared_ptr<mars::sim::SimMotor> newMotor(simMotor);         

      if (attachAndStoreMotor(newMotor, motorS->jointName))
      {
#ifdef DEBUG        
        LOG_DEBUG(("[EnvireMotorManager::addMotor]: Found the joint " + motorS->jointName + " to which the motor " + motorS->name + " should be attached").c_str());
#endif
      }
      else
      {
#ifdef DEBUG        
        LOG_ERROR(("[EnvireMotorManager::addMotor]: Not found the joint " + motorS->jointName + " to which the motor " + motorS->name + " should be attached in any frame ").c_str());
#endif
      }
      newMotor->setSMotor(*motorS);
      iMutex.lock();
      simMotors[newMotor->getIndex()] = newMotor.get();
      iMutex.unlock();
      control->sim->sceneHasChanged(false);

      configmaps::ConfigMap &config = motorS->config;

      // set motor mimics
      if (config.find("mimic_motor") != config.end()) {
        mimicmotors[motorS->index] = (std::string)config["mimic_motor"];
        newMotor->setMimic(
          (sReal)config["mimic_multiplier"], (sReal)config["mimic_offset"]);
      }

      // set approximation functions
      if (config.find("maxeffort_approximation") != config.end()) {
        std::vector<sReal>* maxeffort_coefficients = new std::vector<sReal>;
        ConfigVector::iterator vIt = config["maxeffort_coefficients"].begin();
        for (; vIt != config["maxeffort_coefficients"].end(); ++vIt) {
          maxeffort_coefficients->push_back((double)(*vIt));
          newMotor->setMaxEffortApproximation(
            utils::getApproximationFunctionFromString((std::string)config["maxeffort_approximation"]),
            maxeffort_coefficients);
        }
      }
      if (config.find("maxspeed_approximation") != config.end()) {
        std::vector<sReal>* maxspeed_coefficients = new std::vector<sReal>;
        ConfigVector::iterator vIt = config["maxspeed_coefficients"].begin();
        for (; vIt != config["maxspeed_coefficients"].end(); ++vIt) {
          maxspeed_coefficients->push_back((double)(*vIt));
          newMotor->setMaxSpeedApproximation(
            utils::getApproximationFunctionFromString((std::string)config["maxspeed_approximation"]),
            maxspeed_coefficients);
        }
      }
      if (config.find("current_approximation") != config.end()) {
        std::vector<sReal>* current_coefficients = new std::vector<sReal>;
        ConfigVector::iterator vIt = config["current_coefficients"].begin();
        for (; vIt != config["current_coefficients"].end(); ++vIt) {
          current_coefficients->push_back((double)(*vIt));
          newMotor->setCurrentApproximation(
            utils::getApproximationFunction2DFromString((std::string)config["current_approximation"]),
            current_coefficients);
        }
      }

      return motorS->index;
    }

    /*
     * - Find in the Graph the vertex that contains the record with same name 
     * as the joint. 
     * - Get from that record the simjoint 
     * - Attach the motor to the simjoint
     * - Store the motor
    */
    bool EnvireMotorManager::attachAndStoreMotor(std::shared_ptr<mars::sim::SimMotor> simMotor, const std::string & jointName)
    {

      using VertexIterator = envire::core::EnvireGraph::vertex_iterator;
      using JointRecordItem = envire::core::Item<mars::sim::JointRecord>;
      using JointRecordItemIterator = envire::core::EnvireGraph::ItemIterator<JointRecordItem>;
      using simMotorItemPtr = envire::core::Item<std::shared_ptr<mars::sim::SimMotor>>::Ptr;

      VertexIterator vi_begin, vi_end;
      boost::tie(vi_begin, vi_end) = control->graph->getVertices();
      bool jointFound = false;
      
      while ((vi_begin!=vi_end) && (!jointFound))
      {
        if (control->graph->containsItems<JointRecordItem>(*vi_begin))
        {
          envire::core::FrameId frameName = control->graph->getFrameId(*vi_begin);
          JointRecordItemIterator jri_begin, jri_end;
          boost::tie(jri_begin, jri_end) = control->graph->getItems<JointRecordItem>(frameName); 
          while ((jri_begin!=jri_end) && (!jointFound))
          {
            mars::sim::JointRecord jointRecord = jri_begin->getData();
            if (jointRecord.name == jointName)
            {
              jointFound = true;
              std::shared_ptr<mars::sim::SimJoint> simJoint = jointRecord.sim;
              simMotor->attachJoint(simJoint.get());
              simMotorItemPtr simMotorItem(new envire::core::Item<shared_ptr<mars::sim::SimMotor>>(simMotor));
              control->graph->addItemToFrame(frameName, simMotorItem);
            }
            jri_begin ++;
          }
        }
        vi_begin++;
      }
      return jointFound;
    }

    /**
     *\brief Returns the number of motors that are currently present in the simulation.
     * 
     *\return The number of all motors.
     */
    int EnvireMotorManager::getMotorCount() const {
      MutexLocker locker(&iMutex);
      return simMotors.size();
    }


    /**
     * \brief Change motor properties.
     *
     * \details The old struct is replaced 
     * by the new one completely, so prior to calling this function, one must 
     * ensure that all properties of this parameter are valid and as desired.
     *
     * \param motorS The id of the MotorData referred by this pointer must be the
     * same as the id of the motor that is to be edited. 
     */
    void EnvireMotorManager::editMotor(const MotorData &motorS) {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::iterator iter = simMotors.find(motorS.index);
      if (iter != simMotors.end())
        iter->second->setSMotor(motorS);
    }


    /**
     * \brief Gives information about core exchange data for motors.
     *
     * \param motorList A pointer to a vector that is filled with a
     * core_objects_exchange struct for every motor and its index. The vector is cleared
     * in the beginning of this function.
     */
    void EnvireMotorManager::getListMotors(vector<core_objects_exchange> *motorList)const{
      core_objects_exchange obj;
      map<unsigned long, mars::sim::SimMotor*>::const_iterator iter;
      motorList->clear();
      iMutex.lock();
      for (iter = simMotors.begin(); iter != simMotors.end(); iter++) {
        iter->second->getCoreExchange(&obj);
        motorList->push_back(obj);
      }
      iMutex.unlock();
    }


    /**
     * \brief Gives all information of a certain motor.
     *
     * \param index The unique id of the motor to get information for.
     *
     * \return A pointer to the MotorData of the motor with the given id.
     * \throw std::runtime_error if a motor with the given index does not exist.
     */
    const MotorData EnvireMotorManager::getFullMotor(unsigned long index) const {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::const_iterator iter = simMotors.find(index);
      if (iter != simMotors.end())
        return iter->second->getSMotor();
      else {
        char msg[128];
        sprintf(msg, "could not find motor with index: %lu", index);
        throw std::runtime_error(msg);
      }
    }


    /**
     * \brief Removes a motor from the simulation.
     *
     * \param index The unique id of the motor to remove form the simulation.
     */
    void EnvireMotorManager::removeMotor(unsigned long index) {
      mars::sim::SimMotor* tmpMotor = NULL;
      iMutex.lock();
      map<unsigned long, mars::sim::SimMotor*>::iterator iter = simMotors.find(index);
      if (iter != simMotors.end()) {
        tmpMotor = iter->second;
        simMotors.erase(iter);
        if (tmpMotor)
          delete tmpMotor;
      }
      iMutex.unlock();
  
      control->sim->sceneHasChanged(false);
    }


    /**
     * \brief This function returns the SimMotor object for a given id.
     *
     * \warning This method is only internal used by the
     * EnvireMotorManager. Generally no other modules know the SimMotor class and
     * shouldn't use this method. All motor operations from outside the core
     * should be done over the EnvireMotorManager.
     *
     * \param index The id of the motor to get the core node object.
     *
     * \returns Returns a pointer to the corresponding motor object.
     */
    mars::sim::SimMotor* EnvireMotorManager::getSimMotor(unsigned long id) const {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::const_iterator iter = simMotors.find(id);
      if (iter != simMotors.end())
        return iter->second;
      else
        return NULL;
    }



    /**
     * \brief This function returns the SimMotor object for a given name.
     *
     * \warning This method is only internal used by the
     * EnvireMotorManager. Generally no other modules know the SimMotor class and
     * shouldn't use this method. All motor operations from outside the core
     * should be done over the EnvireMotorManager.
     *
     * \param name The name of the motor to get the core node object.
     *
     * \returns Returns a pointer to the corresponding motor object.
     */
    mars::sim::SimMotor* EnvireMotorManager::getSimMotorByName(const std::string &name) const {
      MutexLocker locker(&iMutex);
      std::map<unsigned long, mars::sim::SimMotor*>::const_iterator iter;
      for (iter = simMotors.begin(); iter != simMotors.end(); iter++)
        if (iter->second->getName() == name)
          return iter->second;
      return NULL;
    }


    /**
     * \brief Sets the value of the motor with the given id to the given value.
     *
     * Essentially this function triggers the motor and moves the joint that is
     * attached to it.
     * Equivalent to \c moveMotor
     *
     * \param id The id of the motor whose value is to be changed.
     *
     * \param value The new value.
     */
    void EnvireMotorManager::setMotorValue(unsigned long id, sReal value) {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::iterator iter = simMotors.find(id);
      if (iter != simMotors.end())
        iter->second->setControlValue(value);
    }


    void EnvireMotorManager::setMotorValueDesiredVelocity(unsigned long id, sReal velocity) {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::iterator iter = simMotors.find(id);
      if (iter != simMotors.end())
        iter->second->setVelocity(velocity);
    }



    /**
     * \brief Sets the proportional term of the motor with the given id to the given value.
     *
     * \details Only has effect on a PID motor. If the type of the motor with
     * the given id is different from PID, no effect is observed, although the 
     * P value of the motor object is still changed.
     *
     * \param id The id of the motor whose P value is to be changed.
     *
     * \param value The new P value.
     */
    void EnvireMotorManager::setMotorP(unsigned long id, sReal value) {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::iterator iter = simMotors.find(id);
      if (iter != simMotors.end())
        iter->second->setP(value);
    }


    /**
     * \brief Sets the integral term of the motor with the given id to the given value.
     *
     * \details Only has effect on a PID motor. If the type of the motor with
     * the given id is different from PID, no effect is observed, although the 
     * I value of the motor object is still changed.
     *
     * \param id The id of the motor whose I value is to be changed.
     *
     * \param value The new I value.
     */
    void EnvireMotorManager::setMotorI(unsigned long id, sReal value) {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::iterator iter = simMotors.find(id);
      if (iter != simMotors.end())
        iter->second->setI(value);
    }


    /**
     * \brief Sets the derivative term of the motor with the given id to the given value.
     *
     * \details Only has effect on a PID motor. If the type of the motor with
     * the given id is different from PID, no effect is observed, although the 
     * D value of the motor object is still changed.
     *
     * \param id The id of the motor whose D value is to be changed.
     *
     * \param value The new D value.
     */
    void EnvireMotorManager::setMotorD(unsigned long id, sReal value) {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::iterator iter = simMotors.find(id);
      if (iter != simMotors.end())
        iter->second->setD(value);
    }


    /** 
     * \brief Deactivates the motor with the given id.
     *
     * \param id The id of the motor that is to be deactivated.
     */
    void EnvireMotorManager::deactivateMotor(unsigned long id) {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::iterator iter = simMotors.find(id);
      if (iter != simMotors.end())
        iter->second->deactivate();
    }


    /**
     * \brief Retrieves the id of a motor by name
     *
     * \param motor_name Name of the motor to get the id for
     *
     * \return Id of the motor if it exists, otherwise 0
     */
    unsigned long EnvireMotorManager::getID(const std::string& name) const {
      map<unsigned long, mars::sim::SimMotor*>::const_iterator iter;
      MutexLocker locker(&iMutex);

      for (iter = simMotors.begin(); iter != simMotors.end(); iter++) {
        if (iter->second->getName() == name)
          return iter->first;
      }
      return 0;
    }


    /**
     * \brief Sets the value of the motor with the given id to the given value.
     *
     * Essentially this function triggers the motor and moves the joint that is
     * attached to it.
     * Equivalent to \c setMotorValue
     *
     * \param id The id of the motor whose value is to be changed.
     *
     * \param value The new value.
     */
    void EnvireMotorManager::moveMotor(unsigned long index, double value) {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::iterator iter = simMotors.find(index);
      if (iter != simMotors.end())
        iter->second->setControlValue(value);
    }


    /** 
     * \brief Destroys all motors in the simulation.
     *
     * \details The \c clear_all flag indicates if the reload motors should
     * be destroyed as well. If set to \c false they are left intact.
     *
     * \param clear_all Indicates if the reload motors should
     * be destroyed as well. If set to \c false they are left intact.
     */
    void EnvireMotorManager::clearAllMotors(bool clear_all) {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::iterator iter;
      for(iter = simMotors.begin(); iter != simMotors.end(); iter++)
        delete iter->second;
      simMotors.clear();
      mimicmotors.clear();
      if(clear_all) simMotorsReload.clear();
      next_motor_id = 1;
    }


    /**
     * \brief This function reloads all motors from a temporary motor pool.
     *
     * \details All motors that have been added with \c reload value as \c true
     * are added back to the simulation again with a \c reload value of \c true. 
     */
    void EnvireMotorManager::reloadMotors(void) {
      list<MotorData>::iterator iter;
      iMutex.lock();
      for(iter = simMotorsReload.begin(); iter != simMotorsReload.end(); iter++) {
        iMutex.unlock();
        addMotor(&(*iter), true);
        iMutex.lock();
      }
      iMutex.unlock();
      connectMimics();
    }

    /**
     * \brief This function updates all motors with timing value \c calc_ms in miliseconds.
     *
     * \warning This function is only used internally and should not be called 
     * outside the core.
     *
     * \param calc_ms The timing value in miliseconds. 
     */
    void EnvireMotorManager::updateMotors(double calc_ms) {
      map<unsigned long, mars::sim::SimMotor*>::iterator iter;
      MutexLocker locker(&iMutex);
      for(iter = simMotors.begin(); iter != simMotors.end(); iter++)
        iter->second->update(calc_ms);
    }


    sReal EnvireMotorManager::getActualPosition(unsigned long motorId) const {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::const_iterator iter;
      iter = simMotors.find(motorId);
      if (iter != simMotors.end())
        return iter->second->getPosition();
      return 0.;
    }

    sReal EnvireMotorManager::getTorque(unsigned long motorId) const {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::const_iterator iter;
      iter = simMotors.find(motorId);
      if (iter != simMotors.end())
        return iter->second->getEffort();
      return 0.;
    }

    void EnvireMotorManager::setMaxTorque(unsigned long id, sReal maxTorque) {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::const_iterator iter;
      iter = simMotors.find(id);
      if (iter != simMotors.end())
        iter->second->setMaxEffort(maxTorque);
    }

    void EnvireMotorManager::setMaxSpeed(unsigned long id, sReal maxSpeed) {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::const_iterator iter;
      iter = simMotors.find(id);
      if (iter != simMotors.end())
        iter->second->setMaxSpeed(maxSpeed);
    }


    /**
     * \brief Detaches the joint with the given index from all motors that act on
     * it, if any. Used when a joint is destroyed.
     * 
     * \warning The detached motors are not destroyed and are still present in the 
     * simulation, although they do not have any effect on it. A call to 
     * \c removeMotor must be made to remove the motor completely.
     *
     * \param joint_index The id of the joint that is to be detached.
     */
    void EnvireMotorManager::removeJointFromMotors(unsigned long joint_index) {
      map<unsigned long, mars::sim::SimMotor*>::iterator iter;
      MutexLocker locker(&iMutex);
      for (iter = simMotors.begin(); iter != simMotors.end(); iter++) 
        if (iter->second->getJointIndex() == joint_index) 
          iter->second->attachJoint(0);
    }

    void EnvireMotorManager::getDataBrokerNames(unsigned long jointId, 
                                          std::string *groupName, 
                                          std::string *dataName) const {
      MutexLocker locker(&iMutex);
      map<unsigned long, mars::sim::SimMotor*>::const_iterator iter = simMotors.find(jointId);
      if(iter != simMotors.end())
        iter->second->getDataBrokerNames(groupName, dataName);
    }

    void EnvireMotorManager::connectMimics() {
      std::map<unsigned long, std::string>::iterator it;
      for (it = mimicmotors.begin(); it!=mimicmotors.end(); ++it) {
        mars::sim::SimMotor* parentmotor = getSimMotorByName(it->second);
        if (parentmotor != NULL)
          parentmotor->addMimic(simMotors[it->first]);
      }
    }

  }
  } // end of namespace sim
} // end of namespace mars
