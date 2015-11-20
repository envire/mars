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
  GraphItemEventDispatcher<Item<smurf::StaticTransformation>::Ptr>::subscribe(control->graph);
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

void GraphPhysics::itemAdded(const TypedItemAddedEvent<Item<smurf::Frame>::Ptr>& e)
{
    LOG_DEBUG("[Envire Physics] ItemAdded event-triggered method: About to create a new node data");
    // I think that the node data is generated in the physics plugin from the smurf::Frame 
    mars::interfaces::NodeData node;
    smurf::Frame link= e.item->getData();
    LOG_DEBUG("[Envire Physics] Smurf frame name: " + link.getName());
    node.init(link.getName()); //Node name
    node.initPrimitive(mars::interfaces::NODE_TYPE_BOX, mars::utils::Vector(0.1, 0.1, 0.1), 0.1);
    node.movable = true;
    setPos(e.frame, node);
    // create an interface object to the physics
    shared_ptr<NodeInterface> physics(PhysicsMapper::newNodePhysics(control->sim->getPhysics()));
    if (physics->createNode(&node)) 
    {
      uuidToPhysics[e.item->getID()] = physics;
    }
    // Should we keep the nodeData object also in the Tree or just delete it?
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

// Joints

void GraphPhysics::join(JointData& jointPhysics, const boost::uuids::uuid& uuid1, const boost::uuids::uuid& uuid2, const boost::uuids::uuid& jointID)
{
    
    if(uuidToPhysics.find(uuid1) != uuidToPhysics.end() &&
       uuidToPhysics.find(uuid2) != uuidToPhysics.end())
    {
      shared_ptr<NodeInterface> node1 = uuidToPhysics[uuid1];
      shared_ptr<NodeInterface> node2 = uuidToPhysics[uuid2];
          
      shared_ptr<JointInterface> jointInterface(PhysicsMapper::newJointPhysics(control->sim->getPhysics()));
      // create the physical node data
      LOG_DEBUG("[Envire Physics] Just before creating the physical joint");
      if(jointInterface->createJoint(&jointPhysics, node1.get(), node2.get()))
      {
	LOG_DEBUG("[Envire Physics] Just after creating the physical joint");
        //remember a pointer to the interface, otherwise it will be deleted.
        uuidToJoints[jointID] = jointInterface;
        control->sim->sceneHasChanged(false);//important, otherwise the joint will be ignored by simulation
      }
      else
      {
        std::cerr << "ERROR: Failed to create joint" << std::endl;
        assert(false);//this only happens if the JointConfigMapItem contains bad data, which it should never do
      }
    }
    else
    {
      std::cerr << "ERROR: Nodes do not exist in map" << std::endl;
      assert(false); //bug somewhere, we shouldnt miss items
    }
}

void GraphPhysics::itemAdded(const TypedItemAddedEvent<mars::sim::JointConfigMapItem::Ptr>& e)
{
  
  JointConfigMapItem::Ptr pItem = e.item;
  JointData jointData;
  if(jointData.fromConfigMap(&pItem->getData(), ""))
  {
    string id1 = pItem->getData().get<string>("itemId1", "");
    string id2 = pItem->getData().get<string>("itemId2", "");
    assert(!id1.empty());
    assert(!id2.empty());
    //the ids of the two items that should be connected by the joint
    boost::uuids::uuid uuid1 = boost::lexical_cast<boost::uuids::uuid>(id1);
    boost::uuids::uuid uuid2 = boost::lexical_cast<boost::uuids::uuid>(id2);
    join(jointData, uuid1, uuid2, e.item->getID());
  }
}

void GraphPhysics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::StaticTransformation>::Ptr>& e)
{
    // Get the physic items in the two connected frames and join them
    LOG_DEBUG( "[Envire Physics] itemAdded: envire::core::Item<smurf::StaticTransformation>::Ptr>");
    smurf::StaticTransformation jointEnvire = e.item->getData();
    string source = jointEnvire.getSourceFrame().getName(); // This is a smurf::Frame, not a envire::core::Frame but the name of the frame should be the same
    string target = jointEnvire.getTargetFrame().getName();
    // Get from that fram the item of type SMURF::Frame and from that one the uuid
    using Iterator = TransformGraph::ItemIterator<Item<smurf::Frame>::Ptr>;
    Iterator begin, end;
    boost::tie(begin, end) = control->graph->getItems<Item<smurf::Frame>::Ptr>(source);
    boost::uuids::uuid uuidSource = boost::lexical_cast<boost::uuids::uuid>((*begin)->getID());
    boost::tie(begin, end) = control->graph->getItems<Item<smurf::Frame>::Ptr>(target);
    boost::uuids::uuid uuidTarget = boost::lexical_cast<boost::uuids::uuid>((*begin)->getID());
    LOG_DEBUG( "[Envire Physics] Ids for the joints created");
    JointData jointPhysics; // We miss some data here
    jointPhysics.init(source+"-"+target, mars::interfaces::JOINT_TYPE_FIXED, 0, 0);
    // jointPhysics, uuidSource, uuidTarget, e.item->getID()
    join(jointPhysics, uuidSource, uuidTarget, e.item->getID());
}
 

void GraphPhysics::itemRemoved(const TypedItemRemovedEvent< mars::sim::JointConfigMapItem::Ptr >& e)
{
  if(uuidToJoints.find(e.item->getID()) != uuidToJoints.end())
  {
    shared_ptr<JointInterface> jointInterface = uuidToJoints[e.item->getID()];
    //removing it from the map will destroy the interface because the map
    //was the only one that knew about the item.
    //This will also remove the joint from ode
    uuidToJoints.erase(e.item->getID());
  }
  else
  {
    std::cerr << "ERROR: Tried to remove a joint that was never added" << std::endl;
    assert(false);
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
  
  //// Uncomment to print the Graph
  ////envire::core::GraphViz viz;
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
  LOG_DEBUG("[Envire Physics] Updating position of physical objects in frame: " + control->graph->getFrame(target).getName());
  LOG_DEBUG("[envire_physics] Transformation to be updated: " +  control->graph->getFrame(origin).getName() + " to " + control->graph->getFrame(target).getName() );
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

