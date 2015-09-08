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

//#include <envire_core/Item.hpp>

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
   
         
    //NodeId ItemManager::addItem(NodeData *nodeS, bool reload,
                                //bool loadGraphics) {
    int ItemManager::addItem() {
      printf("test itemManager\n");

      NodeData node;
      
      itemNodeData.setData(node);
      printf("item set data done\n");

      NodeData *nodeS = &node;

      printf("Pointer assignation\n");

      const std::string &name = "box";
      nodeS->pos = utils::Vector::Zero();
      nodeS->name = name;
      nodeS->movable = true;
      nodeS->physicMode = NODE_TYPE_BOX;
      nodeS->index=1;
      nodeS->noPhysical = false;
      nodeS->inertia_set=false;
      nodeS->ext.x() = 1;
      nodeS->ext.y() = 1; 
      nodeS->ext.z() = 1;   
      NodeType type = nodeS->physicMode;
      nodeS->mass = 1.0f;
      nodeS->initPrimitive(type, nodeS->ext, nodeS->mass);
      
      printf("Node S assignations and calls\n");

      // create a node object
      SimNode *newNode = new SimNode(control, *nodeS);
      printf("Create a node object\n");

      // This is not a NodePhysics but many more things. How do I get the NodePhysics from it?

      // create the physical node data
      if(! (nodeS->noPhysical)){
        printf("Enters if\n");
        // create an interface object to the physics
        // The interface to the physics is already in the itemPhysics
        interfaces::PhysicsInterface* physicsInterface = control->sim->getPhysics();
        printf("Physical interface obtained \n");
        NodeInterface *newNodeInterface = PhysicsMapper::newNodePhysics(physicsInterface);
        printf("...........getPhysics.......\n");
        if (!newNodeInterface->createNode(nodeS)) {
          // if no node was created in physics
          // delete the objects
          delete newNode;
          delete newNodeInterface;
          // and return false
          LOG_ERROR("NodeManager::addNode: No node was created in physics.");
          return INVALID_ID;
        }
        printf("Node Created a Node Physics \n");
        newNode->setInterface(newNodeInterface);
        printf("interface is set \n");
        // Set the node in the item wraper newNodeInterface is the pointer to a
        // NodeInterface which is a more abstract class than NodePhysics,
        // therefore it can not just be attached to an item
        //NodePhysics *nodePhysics = newNodeInterface;
        //itemNodeInterface.setData(newNodeInterface);
        // and include the item in the tree
        // envireTree.addVertex(item);
        // Instead of maps we use the envire Tree
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
          // What is done here?
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
		return 0;							
    }
    
    
   void ItemManager::preGraphicsUpdate() {
	//	printf("...preGraphicsUpdate...\n");
      NodeMap::iterator iter;
      if(!control->graphics)
        return;

      iMutex.lock();
      if(update_all_nodes) {
        update_all_nodes = false;
        for(iter = simNodes.begin(); iter != simNodes.end(); iter++) {
          control->graphics->setDrawObjectPos(iter->second->getGraphicsID(),
                                              iter->second->getVisualPosition());
          control->graphics->setDrawObjectRot(iter->second->getGraphicsID(),
                                              iter->second->getVisualRotation());
          control->graphics->setDrawObjectPos(iter->second->getGraphicsID2(),
                                              iter->second->getPosition());
          control->graphics->setDrawObjectRot(iter->second->getGraphicsID2(),
                                              iter->second->getRotation());
        }
      }
      else {
        for(iter = simNodesDyn.begin(); iter != simNodesDyn.end(); iter++) {
          control->graphics->setDrawObjectPos(iter->second->getGraphicsID(),
                                              iter->second->getVisualPosition());
          control->graphics->setDrawObjectRot(iter->second->getGraphicsID(),
                                              iter->second->getVisualRotation());
          control->graphics->setDrawObjectPos(iter->second->getGraphicsID2(),
                                              iter->second->getPosition());
          control->graphics->setDrawObjectRot(iter->second->getGraphicsID2(),
                                              iter->second->getRotation());
        }
        for(iter = nodesToUpdate.begin(); iter != nodesToUpdate.end(); iter++) {
          control->graphics->setDrawObjectPos(iter->second->getGraphicsID(),
                                              iter->second->getVisualPosition());
          control->graphics->setDrawObjectRot(iter->second->getGraphicsID(),
                                              iter->second->getVisualRotation());
          control->graphics->setDrawObjectPos(iter->second->getGraphicsID2(),
                                              iter->second->getPosition());
          control->graphics->setDrawObjectRot(iter->second->getGraphicsID2(),
                                              iter->second->getRotation());
        }
        nodesToUpdate.clear();
      }
      iMutex.unlock();
    }

       
    void ItemManager::updateItemDynamics(sReal calc_ms, bool physics_thread) {
      MutexLocker locker(&iMutex);
      NodeMap::iterator iter;
      for(iter = simNodesDyn.begin(); iter != simNodesDyn.end(); iter++) {
        iter->second->update(calc_ms, physics_thread);
      }
    } 
    
    const Vector ItemManager::getPosition(NodeId id) const {
      Vector pos(0.0,0.0,0.0);
      MutexLocker locker(&iMutex);
      NodeMap::const_iterator iter = simNodes.find(id);
      if (iter != simNodes.end())
        pos = iter->second->getPosition();
        
  //      printf("pos.z()...... = %f\n", pos.z());
      return pos;
    }


    const Quaternion ItemManager::getRotation(NodeId id) const {
      Quaternion q(Quaternion::Identity());
      MutexLocker locker(&iMutex);
      NodeMap::const_iterator iter = simNodes.find(id);
      if (iter != simNodes.end())
        q = iter->second->getRotation();
      return q;
    }       

  } // end of namespace sim
} // end of namespace mars
