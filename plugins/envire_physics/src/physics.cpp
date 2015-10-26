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


using namespace mars::plugins::envire_physics;
using namespace mars::utils;
using namespace mars::interfaces;
using namespace envire::core;
using namespace mars::sim;
using namespace std;
using namespace base;

GraphPhysics::GraphPhysics(lib_manager::LibManager *theManager)
  : MarsPluginTemplate(theManager, "GraphPhysics"), Dispatcher(*(control->graph)){
}

void GraphPhysics::init() {
  /*
  if(control->graph != nullptr)
  {
    control->graph->subscribe(std::shared_ptr<GraphEventDispatcher>(this));
  }
  */
}

void GraphPhysics::reset() {
}

GraphPhysics::~GraphPhysics() {
  /*
  if(control->graph != nullptr)
  {
    control->graph->unsubscribe(std::shared_ptr<GraphEventDispatcher>(this));
  }
  */
}

void GraphPhysics::frameAdded(const FrameAddedEvent& e)
{
  /*
  //the first frame that is added is the root (for now)
  if(originId.empty())
  {
    originId = e.addedFrame;
  }
  */
}

void GraphPhysics::frameRemoved(const FrameRemovedEvent& e)
{ 
  /*
  //FIXME do something intelligent of the origin gets removed
  assert(e.removedFrame != originId);
  */
}

void GraphPhysics::transformRemoved(const envire::core::TransformRemovedEvent& e)
{

  //Removing a transform can lead to non trivial changes in the tree.
  //Instead of thinking about them we just recalculate the tree.
  //This is fast enough for now.
 // tree = control->graph->getTree(originId);
 
}

void GraphPhysics::transformAdded(const envire::core::TransformAddedEvent& e)
{
  /*
  //dont give a shit about performance for the first iteration
  tree = control->graph->getTree(originId);
  */
}

void GraphPhysics::transformModified(const envire::core::TransformModifiedEvent& e)
{
  /*
  //for the first iteration we ignore transformation changes from outside this plugin
  */
}

void GraphPhysics::itemAdded(const envire::core::ItemAddedEvent& e)
{
  /*
  boost::shared_ptr<ConfigMapItem> pItem;
  if(pItem = boost::dynamic_pointer_cast<ConfigMapItem>(e.item))
  {
    //assert that this item has not been added before
    assert(uuidToPhysics.find(pItem->getID()) == uuidToPhysics.end());
    configmaps::ConfigMap mapAux = pItem->getData();
    if (std::string(mapAux["name"]) != std::string("joint"))
    {
      try
      {         
	//try to convert the item into a node Data
	NodeData node;
	//FIXME fromConfigMap always returns true? There is no way to check
	//      if the object is actually valid?! WTF
	//      Return a false instead of crashing here:
	if(node.fromConfigMap(&pItem->getData(), "")) // Here it breaks with the joint
	{
	  Transform fromOrigin = control->graph->getTransform(originId, e.frame); 
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
    else
    {
      try
      {         
	JointData joint;
	if(joint.fromConfigMap(&pItem->getData(), ""))
	{
	  Transform fromOrigin = control->graph->getTransform(originId, e.frame); 
	  // create an interface object to the physics
	  shared_ptr<JointInterface> physics(PhysicsMapper::newJointPhysics(control->sim->getPhysics()));
	  // Node 1 and node2 I guess are the ones that are being connected
	  
	  
	  //if (physics->createJoint(&joint, &node1, &node2))
	  //{
	  //  uuidToPhysics[pItem->getID()] = physics;
	  //}
	  
	  
	}      
      }
      catch(const UnknownTransformException& ex)
      {
	cerr << ex.what() << endl;
      }
    }
  }
  */
}

void GraphPhysics::update(sReal time_ms) 
{
  /*
  //dfs visit the tree and update all positions
  const vertex_descriptor originDesc = control->graph->vertex(originId);
  updateChildPositions(originDesc, TransformWithCovariance::Identity());
  */
}

void GraphPhysics::updateChildPositions(const vertex_descriptor vertex,
                                        const TransformWithCovariance& frameToRoot)
{
  /*
  if(tree.find(vertex) != tree.end())
  {
    const unordered_set<vertex_descriptor>& children = tree[vertex];
    for(const vertex_descriptor child : children)
    {
      updatePositions(vertex, child, frameToRoot);
    }
  } 
  */
}

void GraphPhysics::updatePositions(const vertex_descriptor origin,
                                   const vertex_descriptor target,
                                   const TransformWithCovariance& originToRoot)
{
  /*
  const vector<ItemBase::Ptr>& items = control->graph->getItems(target);
  Transform tf = control->graph->getTransform(origin, target);
  //take the first physics item in the item list and update the transform
  std::cout << "Enters update positions " << std::endl;
  for(const ItemBase::Ptr item : items)
  {
    const boost::uuids::uuid& id = item->getID();
    if(uuidToPhysics.find(id) != uuidToPhysics.end())
    {
      //found a physics item
      const shared_ptr<NodeInterface> physics = uuidToPhysics[id];
      TransformWithCovariance absolutTransform;
      physics->getPosition(&absolutTransform.translation);
      physics->getRotation(&absolutTransform.orientation);
      tf.setTransform(originToRoot * absolutTransform);
      control->graph->updateTransform(origin, target, tf);
      break; //there should be **no** more than one physics item per frame
    }
  }
  
  updateChildPositions(target, originToRoot * tf.transform.inverse());
  */
}


void GraphPhysics::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) 
{
}

DESTROY_LIB(mars::plugins::envire_physics::GraphPhysics);
CREATE_LIB(mars::plugins::envire_physics::GraphPhysics);

