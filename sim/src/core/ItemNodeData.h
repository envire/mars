/*
 *  Copyright 2015, DFKI GmbH Robotics Innovation Center
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
 *
 */

#ifndef ITEM_NODE_DATA_H
#define ITEM_NODE_DATA_H

#ifdef _PRINT_HEADER_
  #warning "ItemNodeData.h"
#endif

#include "ItemMars.h"
#include <mars/interfaces/NodeData.h>
#include <memory>


using namespace envire::core;
using namespace std;

namespace mars {
  namespace sim {
    class SimNode;
    	  
    class ItemNodeData : public ItemMars<mars::interfaces::NodeData>{
      public:
        ItemNodeData() : simNode(nullptr) {};

        /**Every ItemNodeData has a corresponding node in the simulation.
         * This is a pointer to that node. */
        std::shared_ptr<SimNode> simNode;

    };
  }
}

#endif  // ITEM_NODE_DATA_H
