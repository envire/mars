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
 * \file ItemManager.cpp
 * \author Yong-Ho Yoo
 */

#include "SimNode.h"
#include "SimJoint.h"
#include "ItemManager.h"
#include "JointManager.h"
#include "PhysicsMapper.h"

#include <mars/interfaces/sim/LoadCenter.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/terrainStruct.h>
#include <mars/interfaces/Logging.hpp>

#include <mars/interfaces/utils.h>
#include <mars/utils/mathUtils.h>

#include <stdexcept>

#include <mars/utils/MutexLocker.h>

namespace mars {
  namespace sim {

    using namespace std;
    using namespace utils;
    using namespace interfaces;

    /**
     *\brief Initialization of a new ItemManager
     *
     * pre:
     *     - a pointer to a ControlCenter is needed
     * post:
     *     - 
     */
    ItemManager::ItemManager(ControlCenter *c) : visual_rep(1),control(c)
    {
      if(control->graphics) {
        GraphicsUpdateInterface *gui = static_cast<GraphicsUpdateInterface*>(this);
        control->graphics->addGraphicsUpdateInterface(gui);
      }

    }

    int ItemManager::test(){
		  printf("test itemManager\n");
       NodeData node;
       NodeData *nodeS = &node;
    
       const std::string &name = "box";
       nodeS->pos = utils::Vector::Zero();
       nodeS->name = name;
       nodeS->movable = true;
       nodeS->physicMode = NODE_TYPE_BOX;
       nodeS->index=0;
       nodeS->noPhysical = false;
       nodeS->inertia_set=true;
       nodeS->ext.x() = 1;
       nodeS->ext.y() = 1; 
       nodeS->ext.z() = 1;   
       NodeType type = nodeS->physicMode;
       nodeS->mass = 1.0f;
       nodeS->initPrimitive(type, nodeS->ext, nodeS->mass);             

      // create a node object
      SimNode *newNode = new SimNode(control, *nodeS);

      // create the physical node data
      if(! (nodeS->noPhysical)){
        // create an interface object to the physics
        NodeInterface *newNodeInterface = PhysicsMapper::newNodePhysics(control->sim->getPhysics());
 //printf("...........getPhysics.......\n");       
        if (!newNodeInterface->createNode(nodeS)) {
          // if no node was created in physics
          // delete the objects
          delete newNode;
          delete newNodeInterface;
          // and return false
          LOG_ERROR("NodeManager::addNode: No node was created in physics.");
          return INVALID_ID;
        }
        // put all data to the correct place
        //      newNode->setSNode(*nodeS);
        newNode->setInterface(newNodeInterface);
        iMutex.lock();
        simNodes[nodeS->index] = newNode;
        if (nodeS->movable)
          simNodesDyn[nodeS->index] = newNode;
        iMutex.unlock();
        control->sim->sceneHasChanged(false);
        if(control->graphics) {
          NodeId id = control->graphics->addDrawObject(*nodeS, visual_rep & 1);
          if(id) newNode->setGraphicsID(id);

          //        NEW_NODE_STRUCT(physicalRep);
          NodeData physicalRep;
          physicalRep = *nodeS;
          physicalRep.material = nodeS->material;
          physicalRep.material.exists = 1;
          physicalRep.material.transparency = 0.3;
          physicalRep.visual_offset_pos = Vector(0.0, 0.0, 0.0);
          physicalRep.visual_offset_rot = Quaternion::Identity();
          physicalRep.visual_size = physicalRep.ext;

          if(nodeS->physicMode != NODE_TYPE_TERRAIN) {
            if(nodeS->physicMode != NODE_TYPE_MESH) {
              physicalRep.filename = "PRIMITIVE";
              //physicalRep.filename = nodeS->filename;
              if(nodeS->physicMode > 0 && nodeS->physicMode < NUMBER_OF_NODE_TYPES){
                physicalRep.origName = NodeData::toString(nodeS->physicMode);
              }
            }
            id = control->graphics->addDrawObject(physicalRep, visual_rep & 2);
            if(id) newNode->setGraphicsID2(id);
          }
          newNode->setVisualRep(visual_rep);
        }
      } 	  
		  
		  };

  } // end of namespace sim
} // end of namespace mars
