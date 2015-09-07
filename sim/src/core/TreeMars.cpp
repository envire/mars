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
 * \file TreeMars.cpp
 * \author Raul Dominguez
 *
 * The TreeMars class inherits from the EnvireTree which is used to represent
 * an environment. In this case the environment represented is the simulated
 * one.
 */

#include "ItemNodeData.h"
#include "TreeMars.h"
#include "SimNode.h"
#include "PhysicsMapper.h"
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <assert.h>

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
    TreeMars::TreeMars(ControlCenter *c) : control(c), visual_rep(1)
    {
      if(control->graphics) {
        GraphicsUpdateInterface *gui = static_cast<GraphicsUpdateInterface*>(this);
        control->graphics->addGraphicsUpdateInterface(gui);
      }
    }


    void TreeMars::minimalTest()
    {
      using std::string;
      using utils::Vector;
      using utils::Quaternion;

      NodeData nodeData;
      nodeData.init("box", //name
                    Vector(1,2,3)); //position

      //we want a primitive object, not a mesh or something similar
      nodeData.initPrimitive(NODE_TYPE_BOX,//node type (box, sphere, etc.)
                             Vector(3, 1, 1),//extents (width, height, length)
                             1);//mass

      //simlator object. Manages communication with mars data broker.
      //Note: Creates an internal copy of nodeData
      SimNode *newNode = new SimNode(control, nodeData);

      //Create the physical node data
      NodeInterface *newNodeInterface = PhysicsMapper::newNodePhysics(control->sim->getPhysics());
      //create physics node, based on the values in nodeData
      if(!newNodeInterface->createNode(&nodeData))
      {
        abort();
      }
      //attach the physics node to the simulation node
      newNode->setInterface(newNodeInterface);
      //sceneHasChanged does not seem to have any effect
      //control->sim->sceneHasChanged(true);

      //The GraphicsManager identifies the object by this id
      NodeId id = control->graphics->addDrawObject(nodeData,
                                                   true); //activated: only visible if true, no idea what it means

      //the sim node needs to know the id of the graphical representation
      //because it has to update the representation, e.g. if the extents of the
      //object change
      newNode->setGraphicsID(id);
      newNode->setVisualRep(visual_rep);//FIXME do I need this?

    }
    int TreeMars::test()
    {
      printf("Test TreeMars\n");

      // Create a node data and add it to the tree
      NodeData node;
      ItemNodeData itemNodeData;
      itemNodeData.setData(node);
      printf("item set data done\n");
      NodeData *nodeS = &node;

      // Configure the object in the node
      const std::string &name = "box";
      nodeS->pos = utils::Vector::Zero();
      nodeS->name = name;
      nodeS->movable = true;
      nodeS->physicMode = NODE_TYPE_BOX;
      nodeS->index=0;
      nodeS->noPhysical = false;
      nodeS->inertia_set=true;
      // I guess these are the position coordinates, they should go on a edge
      nodeS->ext.x() = 1;
      nodeS->ext.y() = 1; 
      nodeS->ext.z() = 1;   
      NodeType type = nodeS->physicMode;
      nodeS->mass = 1.0f;
      nodeS->initPrimitive(type, nodeS->ext, nodeS->mass);

      // Create a node object
      SimNode *newNode = new SimNode(control, *nodeS);
      printf("Create a node object\n");

      // Create the physical node data
      if(! (nodeS->noPhysical)){
        // Create an interface object to the physics
        interfaces::PhysicsInterface* physicsInterface = control->sim->getPhysics();
        NodeInterface *newNodeInterface = PhysicsMapper::newNodePhysics(physicsInterface);
        if (!newNodeInterface->createNode(nodeS)) {
          // If no node was created in physics delete the objects
          delete newNode;
          delete newNodeInterface;
          // and return false
          LOG_ERROR("NodeManager::addNode: No node was created in physics.");
          return INVALID_ID;
        }
        newNode->setInterface(newNodeInterface);
        control->sim->sceneHasChanged(false);
        if(control->graphics) {
          NodeId id = control->graphics->addDrawObject(node, visual_rep & 1);
          if(id) newNode->setGraphicsID(id);
          // What is done here?
          NodeData physicalRep;
          physicalRep = *nodeS;
          physicalRep.material = nodeS->material;
          physicalRep.material.exists = 1;
          physicalRep.material.transparency = 0.3;
          physicalRep.visual_offset_pos = Vector(0.0, 0.0, 0.0);
          physicalRep.visual_offset_rot = Quaternion::Identity();
          physicalRep.visual_size = physicalRep.ext;

          if(node.physicMode != NODE_TYPE_TERRAIN) {
            if(node.physicMode != NODE_TYPE_MESH) {
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

    }

  } // NS sim
} // NS mars
