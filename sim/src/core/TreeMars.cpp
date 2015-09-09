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
 * \author Raul Dominguez, Yong-Ho Yoo, Arne Böckmann
 *
 * The TreeMars class inherits from the EnvireTree which is used to represent
 * an environment. In this case the environment represented is the simulated
 * one.
 */

#include "PhysicsMapper.h"
#include <mars/interfaces/utils.h>
#include <mars/utils/MutexLocker.h>

#include "ItemNodeData.h"
#include "TreeMars.h"
#include "SimNode.h"
#include "PhysicsMapper.h"
#include "ItemMars.h"
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <assert.h>

namespace mars {
  namespace sim {

    using namespace std;
    using namespace utils;
    using namespace interfaces;
    using namespace envire::core;
    using boost::intrusive_ptr;

    TreeMars::TreeMars(ControlCenter *c) : control(c), visual_rep(1)
    {
      if(control->graphics) {
        GraphicsUpdateInterface *gui = static_cast<GraphicsUpdateInterface*>(this);
        control->graphics->addGraphicsUpdateInterface(gui);
      }
    }

    void TreeMars::addObject(const string& name, const NodeData& node,
                             const Transform& location)
    {
      intrusive_ptr<ItemNodeData> item(new ItemNodeData());
      item->getData() = node;
      item->simNode = createSimNode(item);
 
      Frame frame(name);
      frame.items.push_back(item);
      //add_vertex creates a copy of the frame.
      //Therefore the frame has to be initialized completely before adding it.
      TransformTree::vertex_descriptor treeNode = add_vertex(frame);

      std::pair<TransformTree::edge_descriptor, bool> result;
	    result = add_edge(getRootNode(), treeNode, location);
      if(!result.second)
      {
        //FIXME edge is already in the graph and has been updated
        //      should this happen?
        abort();
    }
  }

  std::shared_ptr<SimNode> TreeMars::createSimNode(intrusive_ptr<ItemNodeData> ind)
  {
    //This code is adapted from an example by Yong-Ho Yoo which can be found
    //in ItemManager.h/cpp
    const NodeData& node = ind->getData();
    // create a node object
    shared_ptr<SimNode> newNode(new SimNode(control, node));

    // create the physical node data
    if(!node.noPhysical){
      // create an interface object to the physics
      // The interface to the physics is already in the itemPhysics
      NodeInterface *newNodeInterface = PhysicsMapper::newNodePhysics(control->sim->getPhysics());

      //FIXME remove const_cast
      if (!newNodeInterface->createNode(const_cast<NodeData*>(&node))) {
        delete newNodeInterface;
        abort();//FIXME do something more appropriate here
      }
      //newNode takes ownership of newNodeInterface, we do not need to delete it
      newNode->setInterface(newNodeInterface);

      if(control->graphics) {
        //FIXME visual_rep & 1 is bullshit?!
        NodeId id = control->graphics->addDrawObject(node, visual_rep & 1);
        if(id) newNode->setGraphicsID(id); //TODO figure out what the graphics id does

        // What is done here?
        NodeData physicalRep;
        physicalRep = node; //initialize from node
        physicalRep.material = node.material; //FIXME maybe useless because is done one line above?
        physicalRep.material.exists = true;
        physicalRep.material.transparency = 0.3;
        physicalRep.visual_offset_pos = Vector(0.0, 0.0, 0.0);
        physicalRep.visual_offset_rot = Quaternion::Identity();
        physicalRep.visual_size = physicalRep.ext;

        if(node.physicMode != NODE_TYPE_TERRAIN) {
          if(node.physicMode != NODE_TYPE_MESH) {
            physicalRep.filename = "PRIMITIVE";
            //physicalRep.filename = nodeS->filename;
            if(node.physicMode > 0 && node.physicMode < NUMBER_OF_NODE_TYPES){
              physicalRep.origName = NodeData::toString(node.physicMode);
            }
          }
          id = control->graphics->addDrawObject(physicalRep, visual_rep & 2);
          if(id) newNode->setGraphicsID2(id);
        }
        newNode->setVisualRep(visual_rep);
      }
    }
    return newNode;
  }

  void TreeMars::updateItemDynamics(sReal calc_ms, bool physics_thread) {
    printf("update \n");
    //this method is called by the simulator once per simulation step
    envire::core::TransformTree::edge_iterator it, end;
    for(boost::tie(it, end) = edges(); it != end; ++it)
    {
      TransformTree::vertex_descriptor vertex = target(*it);
      Frame& frame = getFrame(vertex);
      for(intrusive_ptr<ItemBase>& i : frame.items)
      {
        //FIXME this cast sucks. Frame should be templated
        intrusive_ptr<ItemNodeData> item = boost::dynamic_pointer_cast<ItemNodeData>(i);
        assert(item != nullptr);//for now item needs to be of type ItemNodeData
        assert(item->simNode != nullptr);//every item needs a sim node (this will probably change in the future)
        if(item->getData().movable)
        {
          item->simNode->update(calc_ms, physics_thread);
        }
      }

      //update corresponding edge in the graph based on the position and location
      //of the first item in the list
      if(!frame.items.empty())
      {
        //FIXME this assumes that all items move as one
        intrusive_ptr<ItemNodeData> item = boost::dynamic_pointer_cast<ItemNodeData>(frame.items[0]);
        assert(item != nullptr);//for now item needs to be of type ItemNodeData
        assert(item->simNode != nullptr);//every item needs a sim node (this will probably change in the future)
        Transform& tf = getTransform(it);
        const base::TransformWithCovariance transform(base::AngleAxisd(item->simNode->getRotation()),
                                                      item->simNode->getPosition());
        tf.setTransform(transform);
      }
    }
  }

  void TreeMars::preGraphicsUpdate()
  {
    printf("graphics\n");
    if(nullptr == control->graphics)
      return;

    envire::core::TransformTree::vertex_iterator it, end;
    for(boost::tie(it, end) = vertices(); it != end; ++it)
    {
      Frame& frame = getFrame(it);
      //FIXME this assumes that all items move together.
      if(!frame.items.empty())
      {
        for(intrusive_ptr<ItemBase>& i : frame.items)
        {
          //FIXME this cast sucks. Frame should be templated
          intrusive_ptr<ItemNodeData> item = boost::dynamic_pointer_cast<ItemNodeData>(i);
          assert(item != nullptr);//for now item needs to be of type ItemNodeData
          assert(item->simNode != nullptr);//every item needs a sim node (this will probably change in the future)
          shared_ptr<SimNode> simNode = item->simNode;
          control->graphics->setDrawObjectPos(simNode->getGraphicsID(),
                                              simNode->getVisualPosition());
          control->graphics->setDrawObjectRot(simNode->getGraphicsID(),
                                              simNode->getVisualRotation());
          control->graphics->setDrawObjectPos(simNode->getGraphicsID2(),
                                              simNode->getPosition());
          control->graphics->setDrawObjectRot(simNode->getGraphicsID2(),
                                              simNode->getRotation());
        }
      }
    }
  }

  } // NS sim
} // NS mars
