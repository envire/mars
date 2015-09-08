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
 * \file ItemManagerInterface.h
 * \author Yong-Ho Yoo \n
 * \brief "ItemManagerInterface" declares the interfaces for all ItemOperations
 * that are used for the communication between the simulation modules.
 *
 * \version 1.0
 * \date 20.08.2015
 */

#ifndef ITEM_MANAGER_INTERFACE_H
#define ITEM_MANAGER_INTERFACE_H

#ifdef _PRINT_HEADER_
  #warning "ItemManagerInterface.h"
#endif

#include "../sensor_bases.h"
#include "../NodeData.h"
#include "../nodeState.h"

#include <mars/utils/Vector.h>
#include <mars/utils/Quaternion.h>

namespace mars {

  namespace sim {
    class SimNode;
  };

  namespace interfaces {

    /**
     * \author Yong-Ho Yoo \n
     * \brief "ItemManagerInterface" declares the interfaces for all ItemOperations
     * that are used for the communication between the simulation modules.
     *
     */
    
     
    class ItemManagerInterface {
    public:
      virtual ~ItemManagerInterface() {}
      //virtual int test() = 0;
      //virtual NodeId addItem(NodeData *nodeS,
                             //bool reload = false,
                             //bool loadGraphics = true) = 0;        
      virtual int addItem(void) = 0;                
      virtual void updateItemDynamics(interfaces::sReal calc_ms, bool physics_thread = true) = 0;
      
      //virtual void setPosition(interfaces::NodeId id, const utils::Vector &pos);
      virtual const utils::Vector getPosition(interfaces::NodeId id) const = 0;
      //virtual void setRotation(interfaces::NodeId id, const utils::Quaternion &rot);
      virtual const utils::Quaternion getRotation(interfaces::NodeId id) const = 0;    
  //    virtual unsigned long getMaxGroupID() = 0;          

      virtual void preGraphicsUpdate(void) = 0;          

    };

  } // end of namespace interfaces
} // end of namespace mars

#endif  // ITEM_MANAGER_INTERFACE_H
