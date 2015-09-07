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


#include "ItemMars.h"
#include <mars/interfaces/NodeData.h>

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

using namespace envire::core;
using namespace std;

namespace mars {
  namespace sim {
    using namespace std;
    using namespace utils;	  
	using namespace interfaces;
	  
    //ItemNodeData::ItemNodeData(ControlCenter *c) : visual_rep(1),control(c)
    //{
      //if(control->graphics) {
        //GraphicsUpdateInterface *gui = static_cast<GraphicsUpdateInterface*>(this);
        //control->graphics->addGraphicsUpdateInterface(gui);
      //}		
    //}
    	  
    //int ItemNodeData::addItem(){

	
      //const std::string &name = "box";
      //user_data.pos = utils::Vector::Zero();
      //user_data.name = name;
      //user_data.movable = true;
      //user_data.physicMode = NODE_TYPE_BOX;
      //user_data.index=0;
      //user_data.noPhysical = false;
      //user_data.inertia_set=true;
      //user_data.ext.x() = 1;
	  //user_data.ext.y() = 1; 
      //user_data.ext.z() = 1;   
      
      //NodeType type = user_data.physicMode;
      //user_data.mass = 1.0f;
      //user_data.initPrimitive(type, user_data.ext, user_data.mass);

      ////printf("NodeData assigned\n");

      //// create a node object
      //SimNode *newNode = new SimNode(control, user_data);
      ////printf("Create a node object\n");

      //// This is not a NodePhysics but many more things. How do I get the NodePhysics from it?

      //// create the physical node data
      //if(! (user_data.noPhysical)){
      //printf("Enters if\n");
      //// create an interface object to the physics
      //// The interface to the physics is already in the itemPhysics
        //interfaces::PhysicsInterface* physicsInterface = control->sim->getPhysics();
        //printf("Physical interface obtained \n");
        //NodeInterface *newNodeInterface = PhysicsMapper::newNodePhysics(physicsInterface);
        //printf("...........getPhysics.......\n");
        //if (!newNodeInterface->createNode(&user_data)) {
          //// if no node was created in physics
          //// delete the objects
          //delete newNode;
          //delete newNodeInterface;
          //// and return false
          //LOG_ERROR("NodeManager::addNode: No node was created in physics.");
          //return INVALID_ID;
        //}
       //printf("Node Created a Node Physics \n");
        //// put all data to the correct place
        ////      newNode->setSNode(*nodeS);
        //newNode->setInterface(newNodeInterface);
        //printf("interface is set \n");
        //// Set the node in the item wraper newNodeInterface is the pointer to a
        //// NodeInterface which is a more abstract class than NodePhysics,
        //// therefore it can not just be attached to an item
        ////NodePhysics *nodePhysics = newNodeInterface;
        ////itemNodeInterface.setData(newNodeInterface);
        //// and include the item in the tree
        //// envireTree.addVertex(item);
        //// Instead of maps we use the envire Tree
        //iMutex.lock();
        //simNodes[user_data.index] = newNode;
        //if (user_data.movable)
          //simNodesDyn[user_data.index] = newNode;
        //iMutex.unlock();
        //control->sim->sceneHasChanged(false);
        //if(control->graphics) {
          //NodeId id = control->graphics->addDrawObject(user_data, visual_rep & 1);
          //if(id) newNode->setGraphicsID(id);

          ////        NEW_NODE_STRUCT(physicalRep);
          //// What is done here?
          //NodeData physicalRep;
          //physicalRep = user_data;
          //physicalRep.material = user_data.material;
          //physicalRep.material.exists = 1;
          //physicalRep.material.transparency = 0.3;
          //physicalRep.visual_offset_pos = Vector(0.0, 0.0, 0.0);
          //physicalRep.visual_offset_rot = Quaternion::Identity();
          //physicalRep.visual_size = physicalRep.ext;

          //if(user_data.physicMode != NODE_TYPE_TERRAIN) {
            //if(user_data.physicMode != NODE_TYPE_MESH) {
              //physicalRep.filename = "PRIMITIVE";
              ////physicalRep.filename = nodeS->filename;
              //if(user_data.physicMode > 0 && user_data.physicMode < NUMBER_OF_NODE_TYPES){
                //physicalRep.origName = NodeData::toString(user_data.physicMode);
              //}
            //}
            //id = control->graphics->addDrawObject(physicalRep, visual_rep & 2);
            //if(id) newNode->setGraphicsID2(id);
          //}
          //newNode->setVisualRep(visual_rep);
        //}
      //} 
      	  
    ///**
     //*\brief Updates the physical values of Item.
     //*/
    //void ItemNodeData::updateItemDynamics(sReal calc_ms, bool physics_thread) {
      //MutexLocker locker(&iMutex);
      //NodeMap::iterator iter;
      //for(iter = simNodesDyn.begin(); iter != simNodesDyn.end(); iter++) {
        //iter->second->update(calc_ms, physics_thread);
      //}
    //}
		
    }
  }
}

