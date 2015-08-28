#ifndef ITEM_MANAGER_H
#define ITEM_MANAGER_H

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
 * \file ItemManager.h
 * \author Yong-Ho Yoo
 */

#ifdef _PRINT_HEADER_
  #warning "ItemManager.h"
#endif

#include <mars/utils/Mutex.h>
#include <mars/interfaces/graphics/GraphicsUpdateInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/ItemManagerInterface.h>
#include "ItemNodeData.h"

namespace mars {
  namespace sim {

    using namespace interfaces;
    class SimJoint;
    class SimNode;

    /**
     * The declaration of the ItemManager class.
     *
     */
    class ItemManager : public interfaces::ItemManagerInterface,
                        public interfaces::GraphicsUpdateInterface {
    public:
      ItemManager(ControlCenter *c); 
      virtual ~ItemManager(){}
      virtual int test( );
    private:
      //interfaces::NodeId next_node_id;
      //bool update_all_nodes;
      //int visual_rep;
      
      interfaces::ControlCenter *control;
      int visual_rep;

      //ItemNodeInterface itemNodeInterface;
      ItemNodeData itemNodeData;


    };

  } // end of namespace sim
} // end of namespace mars

#endif  // ITEM_MANAGER_H
