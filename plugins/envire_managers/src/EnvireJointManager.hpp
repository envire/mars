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
 * \file EnvireJointManager.h
 * \author Malte Roemmermann
 * \brief "JointManager" is the class that manage all joints and their
 * operations and communication between the different modules of the simulation.
 *
 */

#ifndef MARS_PLUGINS_ENVIRE_JOINTMANAGER_H
#define MARS_PLUGINS_ENVIRE_JOINTMANAGER_H

#ifdef _PRINT_HEADER_
  #warning "EnvireJointManager.h"
#endif

#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/JointManagerInterface.h>
#include <mars/utils/Mutex.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>

namespace mars { 
  namespace sim {
    class SimJoint; 
  }
}

namespace mars {
  namespace plugins {
    namespace envire_managers {

    using SimJointItem =  envire::core::Item<std::shared_ptr<mars::sim::SimJoint>>;
    using SimJointItemPtr = SimJointItem::Ptr;              

    typedef std::map<mars::interfaces::NodeId, SimJointItemPtr> JointMap;

    /**
     * The declaration of the JointManager class.
     */
    class EnvireJointManager : public interfaces::JointManagerInterface {
    public:
      EnvireJointManager(mars::interfaces::ControlCenter *c);
      virtual ~EnvireJointManager(){}
      virtual unsigned long addJoint(mars::interfaces::JointData *jointS, bool reload = false);
      virtual int getJointCount();
      virtual void editJoint(mars::interfaces::JointData *jointS);
      virtual void getListJoints(std::vector<mars::interfaces::core_objects_exchange> *jointList);
      virtual void getJointExchange(unsigned long id, mars::interfaces::core_objects_exchange *obj);
      virtual const mars::interfaces::JointData getFullJoint(unsigned long index);
      virtual void removeJoint(unsigned long index);
      virtual void removeJointByIDs(unsigned long id1, unsigned long id2);
      virtual std::shared_ptr<mars::sim::SimJoint> getSimJoint(unsigned long id);
      virtual std::vector<mars::sim::SimJoint*> getSimJoints(void);
      virtual void reattacheJoints(unsigned long node_id);
      virtual void reloadJoints(void);
      virtual void updateJoints(mars::interfaces::sReal calc_ms);
      virtual void clearAllJoints(bool clear_all=false);
      virtual void setReloadJointOffset(unsigned long id, mars::interfaces::sReal offset);
      virtual void setReloadJointAxis(unsigned long id, const mars::utils::Vector &axis);
      virtual void scaleReloadJoints(mars::interfaces::sReal x, mars::interfaces::sReal y, mars::interfaces::sReal z);
      virtual void setJointTorque(unsigned long id, mars::interfaces::sReal torque);
      virtual void changeStepSize(void);
      virtual void setReloadAnchor(unsigned long id, const mars::utils::Vector &anchor);
      virtual void setSDParams(unsigned long id, mars::interfaces::JointData *sJoint);

      virtual void setVelocity(unsigned long id, mars::interfaces::sReal velocity) ;
      virtual void setVelocity2(unsigned long id, mars::interfaces::sReal velocity) ;
      virtual void setForceLimit(unsigned long id, mars::interfaces::sReal max_force,
                                 bool first_axis = 1);

      virtual unsigned long getID(const std::string &joint_name) const;
      virtual bool getDataBrokerNames(unsigned long id, std::string *groupName,
                                      std::string *dataName) const;
      virtual void setOfflineValue(unsigned long id, mars::interfaces::sReal value);

      virtual mars::interfaces::sReal getLowStop(unsigned long id) const;
      virtual mars::interfaces::sReal getHighStop(unsigned long id) const;
      virtual mars::interfaces::sReal getLowStop2(unsigned long id) const;
      virtual mars::interfaces::sReal getHighStop2(unsigned long id) const;
      virtual void setLowStop(unsigned long id, mars::interfaces::sReal lowStop);
      virtual void setHighStop(unsigned long id, mars::interfaces::sReal highStop);
      virtual void setLowStop2(unsigned long id, mars::interfaces::sReal lowStop2);
      virtual void setHighStop2(unsigned long id, mars::interfaces::sReal highStop2);

    private:
      unsigned long next_joint_id;
      JointMap simJoints;
      //std::list<mars::interfaces::JointData> simJointsReload;
      mars::interfaces::ControlCenter *control;
      mutable mars::utils::Mutex iMutex;
      mars::interfaces::JointManagerInterface* getJointInterface(unsigned long node_id);
      std::list<mars::interfaces::JointData>::iterator getReloadJoint(unsigned long id);

    };

  }
  } // end of namespace sim
} // end of namespace mars

#endif  // JOINT_MANAGER_H
