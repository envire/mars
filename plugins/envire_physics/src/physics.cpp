/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
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


#include "physics.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <envire_core/items/Transform.hpp>
#include <envire_core/graph/TransformGraph.hpp>
#include <envire_core/events/ItemAddedEvent.hpp>
#include <envire_core/events/TransformAddedEvent.hpp>
#include <envire_core/events/TransformModifiedEvent.hpp>
#include <envire_core/events/FrameAddedEvent.hpp>
#include <envire_core/events/FrameRemovedEvent.hpp>
#include <envire_core/graph/TransformGraphExceptions.hpp>
#include <mars/sim/ConfigMapItem.h>
#include <mars/sim/PhysicsMapper.h>
#include <mars/interfaces/sim/NodeInterface.h>
#include <base/TransformWithCovariance.hpp>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <cassert>

using namespace mars::plugins::envire_physics;
using namespace mars::utils;
using namespace mars::interfaces;
using namespace envire::core;
using namespace mars::sim;
using namespace std;
using namespace base;

GraphPhysics::GraphPhysics(lib_manager::LibManager *theManager)
  : MarsPluginTemplate(theManager, "GraphPhysics"){
}

void GraphPhysics::init() {
  assert(control->graph != nullptr);
  GraphEventDispatcher::subscribe(control->graph);
  GraphItemEventDispatcher<mars::sim::PhysicsConfigMapItem::Ptr>::subscribe(control->graph);
  GraphItemEventDispatcher<mars::sim::JointConfigMapItem::Ptr>::subscribe(control->graph);
}

void GraphPhysics::reset() {
}

void GraphPhysics::frameAdded(const FrameAddedEvent& e)
{
  //the first frame that is added is the root (for now)
  if(originId.empty())
  {
    originId = e.addedFrame;
  }
}

void GraphPhysics::frameRemoved(const FrameRemovedEvent& e)
{ 
  //FIXME do something intelligent of the origin gets removed
  assert(e.removedFrame != originId); 
}

void GraphPhysics::transformRemoved(const envire::core::TransformRemovedEvent& e)
{
  //Removing a transform can lead to non trivial changes in the tree.
  //Instead of thinking about them we just recalculate the tree.
  //This is fast enough for now.
  tree = control->graph->getTree(originId);
}

void GraphPhysics::transformAdded(const envire::core::TransformAddedEvent& e)
{
  //dont give a shit about performance for the first iteration
  tree = control->graph->getTree(originId);
}

void GraphPhysics::transformModified(const envire::core::TransformModifiedEvent& e)
{
  //for the first iteration we ignore transformation changes from outside this plugin
}


void GraphPhysics::itemAdded(const TypedItemAddedEvent<mars::sim::JointConfigMapItem::Ptr>& e)
{
  /*
  JointConfigMapItem::Ptr pItem = e.item;
  JointData jointData;
  if(jointData.fromConfigMap(&pItem->getData(), ""))
  {
    string id1 = pItem->getData().get("itemId1", "");
    string id2 = pItem->getData().get("itemId1", "");
    assert(!id1.empty());
    assert(!id2.empty());
    //the ids of the two items that should be connected by the joint
    boost::uuids::uuid uuid1 = boost::lexical_cast<boost::uuids::uuid>(id1);
    boost::uuids::uuid uuid2 = boost::lexical_cast<boost::uuids::uuid>(id2);
    
    if(uuidToPhysics.find(uuid1) != uuidToPhysics.end() &&
       uuidToPhysics.find(uuid2) != uuidToPhysics.end())
    {
      shared_ptr<NodeInterface> node1 = uuidToPhysics[uuid1];
      shared_ptr<NodeInterface> node2 = uuidToPhysics[uuid2];
          
      shared_ptr<JointInterface> jointInterface(PhysicsMapper::newJointPhysics(control->sim->getPhysics()));
      // create the physical node data
      if(jointInterface->createJoint(jointData, node1, node2))
      {
      }
      else
      {
        assert(false); //FIXME can this even fail?
      }
    else
    {
      assert(false);//There is a bug somewhere, we shouldn't miss items
    }
    
    uuidToPhysics[pItem->getID()]
    


    
  }
  */
  
}
void GraphPhysics::itemAdded(const TypedItemAddedEvent<PhysicsConfigMapItem::Ptr>& e)
{
  PhysicsConfigMapItem::Ptr pItem = e.item;
  //assert that this item has not been added before
  assert(uuidToPhysics.find(pItem->getID()) == uuidToPhysics.end());

  try
  {         
    //try to convert the item into a node Data
    NodeData node;
    //FIXME fromConfigMap always returns true? There is no way to check
    //      if the object is actually valid?! WTF
    if(node.fromConfigMap(&pItem->getData(), ""))
    {
      Transform fromOrigin;
      if(originId.compare(e.frame) == 0)
      {
        //this special case happens when the graph only contains one frame
        //and items are added to that frame. In that case aksing the graph 
        //for the transformation would cause an exception
        fromOrigin.setTransform(TransformWithCovariance::Identity());
      }
      else
      {
        fromOrigin = control->graph->getTransform(originId, e.frame); 
      }
      
      node.pos = fromOrigin.transform.translation;
      node.rot = fromOrigin.transform.orientation;
      
      // create an interface object to the physics
      shared_ptr<NodeInterface> physics(PhysicsMapper::newNodePhysics(control->sim->getPhysics()));
      if (physics->createNode(&node)) 
      {
        uuidToPhysics[pItem->getID()] = physics;
      }
    }
  }
  catch(const UnknownTransformException& ex)
  {
    cerr << ex.what() << endl;
  }
}

void GraphPhysics::update(sReal time_ms) 
{

  //dfs visit the tree and update all positions.
  //The transforms in the graph are relativ to their parent while the
  //transform from simulation is relativ to the root.
  //The relativ transformations are easy to calculate when dfs visiting the tree.
  const vertex_descriptor originDesc = control->graph->vertex(originId);
  updateChildPositions(originDesc, TransformWithCovariance::Identity());
}

void GraphPhysics::updateChildPositions(const vertex_descriptor vertex,
                                        const TransformWithCovariance& frameToRoot)
{
  if(tree.find(vertex) != tree.end())
  {
    const unordered_set<vertex_descriptor>& children = tree[vertex].children;
    for(const vertex_descriptor child : children)
    {
      updatePositions(vertex, child, frameToRoot);
    }
  } 
}

void GraphPhysics::updatePositions(const vertex_descriptor origin,
                                   const vertex_descriptor target,
                                   const TransformWithCovariance& originToRoot)
{
  using Iterator = TransformGraph::ItemIterator<PhysicsConfigMapItem::Ptr>;
  Iterator begin, end;
  boost::tie(begin, end) = control->graph->getItems<PhysicsConfigMapItem::Ptr>(target);

  Transform tf = control->graph->getTransform(origin, target);
  //take the first physics item in the item list and update the transform
  //There should be only one physics item per frame. But there can be more than
  //one ConfigMapItems. Therefore we have to iterate and find the physics item.
  //FIXME This could be avoided by subclassing the ConfigMapItem
  for(; begin != end; ++begin)
  {
    const boost::uuids::uuid& id = (*begin)->getID();
    if(uuidToPhysics.find(id) != uuidToPhysics.end())
    {
      //found a physics item
      const shared_ptr<NodeInterface> physics = uuidToPhysics[id];
      TransformWithCovariance absolutTransform;
      physics->getPosition(&absolutTransform.translation);
      physics->getRotation(&absolutTransform.orientation);
      tf.setTransform(originToRoot * absolutTransform);
      
      control->graph->updateTransform(origin, target, tf);
      break; //there should be only one physics item per frame
    }
  }
  
  updateChildPositions(target, originToRoot * tf.transform.inverse());
}


void GraphPhysics::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) 
{
}

DESTROY_LIB(mars::plugins::envire_physics::GraphPhysics);
CREATE_LIB(mars::plugins::envire_physics::GraphPhysics);

