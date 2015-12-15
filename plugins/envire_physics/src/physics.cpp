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
#include <base/TransformWithCovariance.hpp>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <cassert>
#include <boost/lexical_cast.hpp>
#include <envire_core/graph/GraphViz.hpp>


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
  GraphItemEventDispatcher<Item<smurf::Frame>::Ptr>::subscribe(control->graph);
  GraphItemEventDispatcher<Item<urdf::Collision>::Ptr>::subscribe(control->graph);
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

void GraphPhysics::updateTree()
{
  treeView = control->graph->getTree(originId);
  if(treeView.crossEdges.size() > 0)
  {
    const vertex_descriptor source = control->graph->source(treeView.crossEdges[0]);
    const vertex_descriptor target = control->graph->target(treeView.crossEdges[0]);
    const FrameId sourceId = control->graph->getFrameId(source);
    const FrameId targetId = control->graph->getFrameId(target);
    const string msg = "Loop in tree detected: " + sourceId + " --> " + targetId +
                       ". The physics plugin cannot handle loops in the graph";
    throw std::runtime_error(msg);
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
  updateTree();
}

void GraphPhysics::transformAdded(const envire::core::TransformAddedEvent& e)
{
  //dont give a shit about performance for the first iteration
  updateTree();
}

void GraphPhysics::transformModified(const envire::core::TransformModifiedEvent& e)
{
  //for the first iteration we ignore transformation changes from outside this plugin
}

void GraphPhysics::setPos(const envire::core::FrameId& frame, mars::interfaces::NodeData& node)
{
  Transform fromOrigin;
  if(originId.compare(frame) == 0)
  {
    //this special case happens when the graph only contains one frame
    //and items are added to that frame. In that case aksing the graph 
    //for the transformation would cause an exception
    fromOrigin.setTransform(TransformWithCovariance::Identity());
  }
  else
  {
    fromOrigin = control->graph->getTransform(originId, frame); 
  }
  node.pos = fromOrigin.transform.translation;
  node.rot = fromOrigin.transform.orientation;
}   

/*
  1. Load a in a config map the data from the SMURF (SMURF::handleCollision) 
  TODO In the geometry object we don't have any of the contact information
  TODO Inertia information is neither there
  TODO Set moveable somewhere. I guess this happens within the Inertia stuff
  
  2. Dump this configMap in a NodeData with NodeData::fromConfigMap
  
  3. Instantiate the physical object see NodePhysics::createNode(node)
  
  4. Add the created node physics to the graph
*/
NodeData GraphPhysics::getNode(const urdf::Collision& collision, const envire::core::FrameId& frame) {
  NodeData node;
  node.init(collision.name);
  node.fromGeometry(collision.geometry);
  setPos(frame, (node));
  node.movable = true;
  return node;
}

/*
* Create the physical objects and save them in the Graph
* We add the shared_ptr of the physical node interface to access from other plugins
* 
*/
void GraphPhysics::instantiateNode(NodeData node, const envire::core::FrameId& frame, const boost::uuids::uuid uniqueID)
{
  shared_ptr<NodeInterface> physics(PhysicsMapper::newNodePhysics(control->sim->getPhysics()));
  if (physics->createNode(&node))
  {
    uuidToPhysics[uniqueID] = physics;
  }
  // TODO: Save the nodePhysics in the graph somehow
  //using physicsItemPtr = envire::core::Item<NodeInterface>::Ptr;
  //physicsItemPtr physicsItem(new envire::core::Item<NodeInterface>(physics));
  //control->graph->addItemToFrame(frame, physicsItem);
}

/**
 * TODO:
 * 
 * 
 */
void GraphPhysics::itemAdded(const TypedItemAddedEvent<Item<urdf::Collision>::Ptr>& e)
{
  //LOG_DEBUG("[Envire Physics] ItemAdded event-triggered method: About to create a new node data");
  // TODO: Can be more efficient by making in a single loop the getCollidables and the instantiate nodes. But this is more clear to understand.
  urdf::Collision collision = e.item->getData();
  NodeData collisionNode = getNode(collision, e.frame);
  instantiateNode(collisionNode, e.frame, e.item->getID());
}

/** 
 * The nodes created by the storage of a smurf Frame are the ones the joints link
 * 
 */
void GraphPhysics::itemAdded(const TypedItemAddedEvent<Item<smurf::Frame>::Ptr>& e)
{
    mars::interfaces::NodeData node;
    smurf::Frame link = e.item->getData();
    node.init(link.getName());
    node.initPrimitive(mars::interfaces::NODE_TYPE_BOX, mars::utils::Vector(0.1, 0.1, 0.1), 0.1);
    node.movable = true;
    setPos(e.frame, node);
    shared_ptr<NodeInterface> physics(PhysicsMapper::newNodePhysics(control->sim->getPhysics()));
    if (physics->createNode(&node)) 
    {
      uuidToPhysics[e.item->getID()] = physics;
    }
    using physicsItemPtr = envire::core::Item<std::shared_ptr<NodeInterface>>::Ptr;
    physicsItemPtr physicsItem(new envire::core::Item<std::shared_ptr<NodeInterface>>(physics));
    control->graph->addItemToFrame(e.frame, physicsItem);
    //LOG_DEBUG("[Envire Physics] ItemAdded event smurf::Frame - an item containing share_ptr to a nodeInterface was stored in " + e.frame);
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
  
  
  // Uncomment to print the Graph
  //envire::core::GraphViz viz;
  //std::string timeStamp = base::Time::now().toString();
  //std::string name = "BeforeUpdatePhysics" + timeStamp + ".dot";
  //viz.write(*(control->graph), name);
  
  
  updateChildPositions<Item<smurf::Frame>>(originDesc, TransformWithCovariance::Identity());
  updateChildPositions<PhysicsConfigMapItem>(originDesc, TransformWithCovariance::Identity());
  updateChildPositions<Item<urdf::Collision>>(originDesc, TransformWithCovariance::Identity()); //Not sure of this...
  
  
  // Uncomment to print the Graph
  //envire::core::GraphViz viz;
  //timeStamp = base::Time::now().toString();
  //name = "AfterUpdatePhysicsConfigs" + timeStamp + ".dot";
  //viz.write(*(control->graph), name);
  
  
  
}

template <class physicsType>void GraphPhysics::updateChildPositions(const vertex_descriptor vertex,
                                                                    const TransformWithCovariance& frameToRoot)
{
  // Perform updatePositions for each of your childs
  if(treeView.tree.find(vertex) != treeView.tree.end())
  {
    const unordered_set<vertex_descriptor>& children = treeView.tree[vertex].children;
    for(const vertex_descriptor child : children)
    {
      updatePositions<physicsType>(vertex, child, frameToRoot);
    }
  } 
}

template <class physicsType> void GraphPhysics::updatePositions( const vertex_descriptor origin,
                                                                 const vertex_descriptor target,
                                                                 const TransformWithCovariance& originToRoot)
{
  Transform tf = control->graph->getTransform(origin, target);
  //LOG_DEBUG("[Envire Physics] Updating position of physical objects in frame: " + control->graph->getFrame(target).getName());
  //LOG_DEBUG("[envire_physics] Transformation to be updated: " +  control->graph->getFrame(origin).getName() + " to " + control->graph->getFrame(target).getName() );
  //How can I print the tf also using the LOG_DEBUG?
  //LOG_DEBUG("[Envire Physics] Tf values before update: " );
  //std::cout << tf.transform << std::endl;
  if (control->graph->containsItems<typename physicsType::Ptr>(target))
  {
    //LOG_DEBUG("[envire_physics] Tf from origin (of the tf to be updated) to root (of the tree): " );
    //std::cout << originToRoot << std::endl;
    using Iterator = TransformGraph::ItemIterator<typename physicsType::Ptr>;
    Iterator begin, end;
    boost::tie(begin, end) = control->graph->getItems<typename physicsType::Ptr>(target);
    const boost::uuids::uuid& id = (*begin)->getID();
    if(uuidToPhysics.find(id) != uuidToPhysics.end())
    {
      //found a physics item
      const shared_ptr<NodeInterface> physics = uuidToPhysics[id];
      TransformWithCovariance absolutTransform;
      physics->getPosition(&absolutTransform.translation);
      physics->getRotation(&absolutTransform.orientation);
      tf.setTransform(originToRoot * absolutTransform);
      //LOG_DEBUG("[Envire Physics] AbsolutTransform, provided by the physical engine: ");
      //std::cout << absolutTransform << std::endl;
      //LOG_DEBUG("[Envire Physics] Final updated transform = AbsolutTransform*origiToRoot: ");
      //std::cout << tf.transform << std::endl;
      control->graph->updateTransform(origin, target, tf);
    }
  }
  updateChildPositions<physicsType>(target, tf.transform.inverse()*originToRoot);
}

void GraphPhysics::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) 
{
}

DESTROY_LIB(mars::plugins::envire_physics::GraphPhysics);
CREATE_LIB(mars::plugins::envire_physics::GraphPhysics);

