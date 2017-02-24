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
#include "EnvirePhysics.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/terrainStruct.h>
#include <mars/interfaces/Logging.hpp>
#include <mars/sim/ConfigMapItem.h>
#include <mars/sim/PhysicsMapper.h>
#include <mars/sim/SimNode.h>

#include <base/TransformWithCovariance.hpp>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <cassert>
#include <boost/lexical_cast.hpp>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphViz.hpp>

#include <envire_collider_mls/MLSCollision.hpp>
#include <fstream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>

using namespace mars::plugins::envire_physics;
using namespace mars::utils;
using namespace mars::interfaces;
using namespace envire::core;
using namespace mars::sim;
using namespace std;
using namespace base;

EnvirePhysics::EnvirePhysics(lib_manager::LibManager *theManager)
  : MarsPluginTemplate(theManager, "EnvirePhysics"){
}

void EnvirePhysics::init() {
  assert(control->graph != nullptr);
  GraphEventDispatcher::subscribe(control->graph.get());
  GraphItemEventDispatcher<Item<configmaps::ConfigMap>>::subscribe(control->graph.get());
  GraphItemEventDispatcher<mars::sim::PhysicsConfigMapItem>::subscribe(control->graph.get());
  GraphItemEventDispatcher<mars::sim::JointConfigMapItem>::subscribe(control->graph.get());
  GraphItemEventDispatcher<Item<smurf::Frame>>::subscribe(control->graph.get());
  GraphItemEventDispatcher<Item<urdf::Collision>>::subscribe(control->graph.get());
  GraphItemEventDispatcher<Item<smurf::Collidable>>::subscribe(control->graph.get());
  GraphItemEventDispatcher<Item<smurf::Inertial>>::subscribe(control->graph.get());
  GraphItemEventDispatcher<Item<NodeData>>::subscribe(control->graph.get());
#ifdef DEBUG
  LOG_DEBUG("[EnvirePhysics::init] ");
#endif
}

void EnvirePhysics::reset() {
}

void EnvirePhysics::frameAdded(const FrameAddedEvent& e)
{
#ifdef DEBUG
  LOG_DEBUG("[EnvirePhysics::frameAdded] FrameAddedEvent");
#endif        
  //the first frame that is added is the root (for now)
  if(originId.empty())
  {
    originId = e.frame;
  }
}

void EnvirePhysics::frameRemoved(const FrameRemovedEvent& e)
{ 
#ifdef DEBUG
  LOG_DEBUG("[EnvirePhysics::frameRemoved] FrameRemovedEvent");
#endif      
  //FIXME do something intelligent of the origin gets removed
  assert(e.frame != originId); 
}

void EnvirePhysics::edgeRemoved(const envire::core::EdgeRemovedEvent& e)
{
#ifdef DEBUG
  LOG_DEBUG("[EnvirePhysics::edgeRemoved] EdgeRemovedEvent");
#endif    
  //Removing a transform can lead to non trivial changes in the tree.
  //Instead of thinking about them we just recalculate the tree.
  //This is fast enough for now.
  updateTree();
}

void EnvirePhysics::edgeAdded(const envire::core::EdgeAddedEvent& e)
{
#ifdef DEBUG
  LOG_DEBUG("[EnvirePhysics::edgeAdded] EdgeAddedEvent");
#endif  
  //dont give a shit about performance for the first iteration
  updateTree();
}

void EnvirePhysics::edgeModified(const envire::core::EdgeModifiedEvent& e)
{
#ifdef DEBUG
  LOG_DEBUG("[EnvirePhysics::edgeModified] EdgeModifiedEvent");
#endif
  //for the first iteration we ignore transformation changes from outside this plugin
}

void EnvirePhysics::itemAdded(const TypedItemAddedEvent<Item<smurf::Frame>>& e)
{
#ifdef DEBUG
  LOG_DEBUG(("[EnvirePhysics::ItemAdded] Smurf::Frame item received in frame *** " + e.frame + "***").c_str());
#endif
    mars::interfaces::NodeData* node = new mars::interfaces::NodeData;
    std::shared_ptr<NodeData> nodeDataPtr(node);
    smurf::Frame link = e.item->getData();
    nodeDataPtr->init(link.getName());
    nodeDataPtr->initPrimitive(mars::interfaces::NODE_TYPE_BOX, mars::utils::Vector(0.5, 0.5, 0.5), 0.00001);
    nodeDataPtr->c_params.coll_bitmask = 0;
    nodeDataPtr->movable = true;
    nodeDataPtr->groupID = link.getGroupId();
    nodeDataPtr->density = 0.0;
    setPos(e.frame, nodeDataPtr);
    if (instantiateNode(nodeDataPtr, e.frame))
    {
#ifdef DEBUG
      LOG_DEBUG(("[EnvirePhysics::ItemAdded] Smurf::Frame - Instantiated the nodeInterface in frame ***" + e.frame + "***").c_str());
#endif
    }
}

void EnvirePhysics::itemAdded(const TypedItemAddedEvent<Item<smurf::Collidable>>& e)
{
#ifdef DEBUG
  LOG_DEBUG(("[EnvirePhysics::ItemAdded] smurf::Collidable item received in frame *** " + e.frame + "***").c_str());
#endif
  smurf::Collidable collidable = e.item->getData();
  std::shared_ptr<NodeData> collisionNodePtr = getCollidableNode(collidable, e.frame);
  if (instantiateNode(collisionNodePtr, e.frame))
  {
#ifdef DEBUG
      LOG_DEBUG(("[EnvirePhysics::ItemAdded] Smurf::Collidable - Instantiated and stored the nodeInterface correspondent to the collidable in frame ***" + e.frame +"***").c_str());
#endif
  }
}

void EnvirePhysics::itemAdded(const TypedItemAddedEvent<Item<smurf::Inertial>>& e)
{
#ifdef DEBUG
  LOG_DEBUG(("[EnvirePhysics::ItemAdded] smurf::Inertial item received in frame *** " + e.frame + "***").c_str());
#endif
  smurf::Inertial inertial = e.item->getData();
  std::shared_ptr<NodeData> inertialNodePtr = getInertialNode(inertial, e.frame);
  if (instantiateNode(inertialNodePtr, e.frame))
  {
#ifdef DEBUG
    LOG_DEBUG(("[EnvirePhysics::ItemAdded] Smurf::Inertial - Instantiated and Stored the nodeInterface in frame ***" + e.frame +"***").c_str());
#endif
  } 
}

void EnvirePhysics::itemAdded(const TypedItemAddedEvent<Item<urdf::Collision>>& e)
{
#ifdef DEBUG
  LOG_DEBUG(("[EnvirePhysics::ItemAdded] urdf::Collision item received in frame *** " + e.frame + "***").c_str());
#endif
  urdf::Collision collision = e.item->getData();
  NodeData * node = new NodeData;
  node->init(collision.name);
  node->fromGeometry(collision.geometry);
  node->mass = 0.00001;
  node->density = 0.0;
  std::shared_ptr<NodeData> nodePtr(node);
  setPos(e.frame, nodePtr);
  node->movable = true;
  if (instantiateNode(nodePtr, e.frame))
  {
#ifdef DEBUG    
    LOG_DEBUG(("[EnvirePhysics::ItemAdded] Smurf::Collision - Instantiated and stored the nodeInterface in frame ***" + e.frame +"***").c_str());
#endif
  }
}

void EnvirePhysics::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<configmaps::ConfigMap>>& e)
{
#ifdef DEBUG
  LOG_DEBUG(("[EnvirePhysics::ItemAdded] configmaps::ConfigMap item received in frame *** " + e.frame + "***").c_str());
#endif
  configmaps::ConfigMap configMap = e.item->getData();
  try
  {         
    //try to convert the item into a node Data
    NodeData* node = new NodeData;
    std::shared_ptr<NodeData> nodePtr(node);
    if(node->fromConfigMap(&(configMap), ""))
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
      node->pos = fromOrigin.transform.translation;
      node->rot = fromOrigin.transform.orientation;
      if (instantiateNode(nodePtr, e.frame))
      {
#ifdef DEBUG
        LOG_DEBUG(("[EnvirePhysics::ItemAdded] PhysicsConfigMapItem - Instantiated and stored the nodeInterface in frame ***" + e.frame + "***").c_str());
#endif
      }
    }
  }
  catch(const UnknownTransformException& ex)
  {
    cerr << ex.what() << endl;
  }
}   

void EnvirePhysics::itemAdded(const TypedItemAddedEvent<PhysicsConfigMapItem>& e)
{
#ifdef DEBUG
  LOG_DEBUG(("[EnvirePhysics::ItemAdded] PhysicsConfigMapItem item received in frame *** " + e.frame + "***").c_str());
#endif
  PhysicsConfigMapItem::Ptr pItem = e.item;
  try
  {         
    //try to convert the item into a node Data
    NodeData * node = new NodeData;
    std::shared_ptr<NodeData> nodePtr(node);
    if(node->fromConfigMap(&pItem->getData(), ""))
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
      node->pos = fromOrigin.transform.translation;
      node->rot = fromOrigin.transform.orientation;
      if (instantiateNode(nodePtr, e.frame))
      {
#ifdef DEBUG
        LOG_DEBUG(("[EnvirePhysics::ItemAdded] PhysicsConfigMapItem - Instantiated and stored the nodeInterface in frame ***" + e.frame + "***").c_str());
#endif
      }
    }
  }
  catch(const UnknownTransformException& ex)
  {
    cerr << ex.what() << endl;
  }
}


void EnvirePhysics::itemAdded(const TypedItemAddedEvent<Item<NodeData>>& e)
{
#ifdef DEBUG
  LOG_DEBUG(("[EnvirePhysics::ItemAdded] NodeData item received in frame *** " + e.frame + "***").c_str());
#endif
  Item<NodeData>::Ptr pItem = e.item; 
  NodeData node = pItem->getData();

  //geom_data* gd = (geom_data*)node.data;

  std::shared_ptr<NodeData> nodePtr(&node);
  if (instantiateNode(nodePtr, e.frame))
  {
#ifdef DEBUG
    LOG_DEBUG(("[EnvirePhysics::ItemAdded] Smurf::Inertial - Instantiated and Stored the nodeInterface in frame ***" + e.frame +"***").c_str());
#endif
  } 

}

void EnvirePhysics::update(sReal time_ms) 
{
  const GraphTraits::vertex_descriptor originDesc = control->graph->vertex(originId);
  if(printGraph)
  {
    envire::core::GraphViz viz;
    std::string timeStamp = base::Time::now().toString();
    std::string name = "BeforeUpdatePhysics" + timeStamp + ".dot";
    viz.write(*(control->graph), name);
  }
  updateChildPositions(originDesc, TransformWithCovariance::Identity()); 
  if(printGraph)
  {
    envire::core::GraphViz viz;
    std::string timeStamp = base::Time::now().toString();
    std::string name = "AfterUpdatePhysicsConfigs" + timeStamp + ".dot";
    viz.write(*(control->graph), name);
  }


  /*
   * TODO Remove this
  //updateChildPositions<Item<smurf::Frame>>(originDesc, TransformWithCovariance::Identity());
  //updateChildPositions<PhysicsConfigMapItem>(originDesc, TransformWithCovariance::Identity());
  ////updateChildPositions<Item<urdf::Collision>>(originDesc, TransformWithCovariance::Identity()); 
  //updateChildPositions<Item<smurf::Collidable>>(originDesc, TransformWithCovariance::Identity()); 
  //updateChildPositions<Item<smurf::Inertial>>(originDesc, TransformWithCovariance::Identity()); 
  */
}


void EnvirePhysics::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) 
{
}

// Private Methods

void EnvirePhysics::updateTree()
{
  treeView = control->graph->getTree(originId);
  if(treeView.crossEdges.size() > 0)
  {
    const GraphTraits::vertex_descriptor source = control->graph->source(treeView.crossEdges[0].edge);
    const GraphTraits::vertex_descriptor target = control->graph->target(treeView.crossEdges[0].edge);
    const FrameId sourceId = control->graph->getFrameId(source);
    const FrameId targetId = control->graph->getFrameId(target);
    const string msg = "Loop in tree detected: " + sourceId + " --> " + targetId +
                       ". The physics plugin cannot handle loops in the graph";
    throw std::runtime_error(msg);
  }
}

std::shared_ptr<NodeData> EnvirePhysics::getCollidableNode(const smurf::Collidable& collidable, const envire::core::FrameId& frame) {
  NodeData * node = new NodeData;
  std::shared_ptr<NodeData> nodePtr(node);
  urdf::Collision collision = collidable.getCollision();
  node->init(collision.name);
  node->fromGeometry(collision.geometry);
  node->density = 0.0;
  node->mass = 0.00001;
  setPos(frame, nodePtr);
  node->movable = true;
  node->c_params.fromSmurfCP(collidable.getContactParams());
  node->groupID = collidable.getGroupId();
  return nodePtr;
}


std::shared_ptr<NodeData> EnvirePhysics::getInertialNode(const smurf::Inertial& inertial, const envire::core::FrameId& frame)
{
  NodeData * result = new NodeData;
  std::shared_ptr<NodeData> resultPtr(result);
  urdf::Inertial inertialUrdf = inertial.getUrdfInertial();
  result->init(inertial.getName());
  result->initPrimitive(mars::interfaces::NODE_TYPE_SPHERE, mars::utils::Vector(0.1, 0.1, 0.1), inertialUrdf.mass);
  result->groupID = inertial.getGroupId();
  result->movable = true;
  result->inertia[0][0] = inertialUrdf.ixx;
  result->inertia[0][1] = inertialUrdf.ixy;
  result->inertia[0][2] = inertialUrdf.ixz;
  result->inertia[1][0] = inertialUrdf.ixy;
  result->inertia[1][1] = inertialUrdf.iyy;
  result->inertia[1][2] = inertialUrdf.iyz;
  result->inertia[2][0] = inertialUrdf.ixz;
  result->inertia[2][1] = inertialUrdf.iyz;
  result->inertia[2][2] = inertialUrdf.izz;
  result->inertia_set = true;
  result->c_params.coll_bitmask = 0;
#ifdef DEBUG
  LOG_DEBUG("[EnvirePhysics::getInertialNode] Inertial object's mass: %f", result->mass);
#endif
  result->density = 0.0;
  setPos(frame, resultPtr);
  return resultPtr;
}

bool EnvirePhysics::instantiateNode(const std::shared_ptr<NodeData> &node, const envire::core::FrameId& frame)
{
#ifdef DEBUG
  LOG_DEBUG(("[EnvirePhysics::instantiateNode] create SimNode for NodeData in frame *** " + frame + "***").c_str());
#endif  

  NodeInterface* node_physics = PhysicsMapper::newNodePhysics(control->sim->getPhysics());

  bool instantiated = false;

  if(node->physicMode == NODE_TYPE_MLS) 
    instantiated = addMlsSurface(node.get());
  else 
    instantiated = (node_physics->createNode(node.get()));

  if (instantiated)
  {
#ifdef DEBUG
    LOG_DEBUG("[EnvirePhysics::InstantiateNode] ADD SIM NODE TO " + frame);
#endif
    // Store the physics NodeInterface
    /*using physicsItemPtr = envire::core::Item<shared_ptr<NodeInterface>>::Ptr;
    physicsItemPtr physicsItem(new envire::core::Item<shared_ptr<NodeInterface>>(physics));
    control->graph->addItemToFrame(frame, physicsItem);*/
    // Store the Nodedata
    /*using dataItemPtr = envire::core::Item<shared_ptr<NodeData>>::Ptr;
    dataItemPtr dataItem(new envire::core::Item<shared_ptr<NodeData>>(node));
    control->graph->addItemToFrame(frame, dataItem);*/
    
    //NOTE Create and store also a simNode. The simNode interface is set to the physics node
    mars::sim::SimNode * simNode = new mars::sim::SimNode(control, (*node)); 
    simNode->setInterface(node_physics);
    std::shared_ptr<mars::sim::SimNode> simNodePtr(simNode);
    using SimNodeItemPtr = envire::core::Item<std::shared_ptr<mars::sim::SimNode>>::Ptr;
    using SimNodeItem =  envire::core::Item<std::shared_ptr<mars::sim::SimNode>>;
    SimNodeItemPtr simNodeItem( new SimNodeItem(simNodePtr));        
    control->graph->addItemToFrame(frame, simNodeItem);
#ifdef DEBUG
    LOG_DEBUG("[EnvirePhysics::InstantiateNode] The SimNode is created and added to the graph");
#endif
    
  }
  return instantiated;
}


//struct NullDeleter {template<typename T> void operator()(T*) {}};   
bool EnvirePhysics::addMlsSurface(NodeData* node)
{
   
  WorldPhysics *theWorld = (WorldPhysics*)control->sim->getPhysics();
  current_space = theWorld->getSpace();
  current_space->add ((dGeomID)node->g_mls);  
 
  return true;
} 	


void EnvirePhysics::setPos(const envire::core::FrameId& frame, const std::shared_ptr<mars::interfaces::NodeData>& node)
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
  node->pos = fromOrigin.transform.translation;
  node->rot = fromOrigin.transform.orientation;
}   

void EnvirePhysics::updateChildPositions(const GraphTraits::vertex_descriptor vertex,
                                        const TransformWithCovariance& frameToRoot)
{
  if(treeView.tree.find(vertex) != treeView.tree.end())
  {
    const unordered_set<GraphTraits::vertex_descriptor>& children = treeView.tree[vertex].children;
    for(const GraphTraits::vertex_descriptor child : children)
    {
      updatePositions(vertex, child, frameToRoot);
    }
  } 
}

void EnvirePhysics::updatePositions( const GraphTraits::vertex_descriptor origin,
                                    const GraphTraits::vertex_descriptor target,
                                    const TransformWithCovariance& originToRoot)
{
#ifdef DEBUG  
  LOG_DEBUG("EnvirePhysics::updatePositions");
#endif

  Transform tf = control->graph->getTransform(origin, target);

  if (control->graph->containsItems<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>(target))
  {
    // Update simulation node
    using simNodeType = envire::core::Item<std::shared_ptr<mars::sim::SimNode>>;
    using IteratorSimNode = EnvireGraph::ItemIterator<simNodeType>;
    IteratorSimNode begin_sim, end_sim;
    boost::tie(begin_sim, end_sim) = control->graph->getItems<simNodeType>(target);
    double calc_ms = control->sim->getCalcMs();
    for (;begin_sim!=end_sim; begin_sim++)
    {
      const std::shared_ptr<mars::sim::SimNode> sim_node = begin_sim->getData();
      sim_node->update(calc_ms);

      TransformWithCovariance absolutTransform;
      absolutTransform.translation = sim_node->getPosition();
      absolutTransform.orientation = sim_node->getRotation();    

      tf.setTransform(originToRoot * absolutTransform); 
      control->graph->updateTransform(origin, target, tf);
    }
  }

  const Transform invTf = control->graph->getTransform(target, origin);
  updateChildPositions(target, invTf.transform * originToRoot);
}

DESTROY_LIB(mars::plugins::envire_physics::EnvirePhysics);
CREATE_LIB(mars::plugins::envire_physics::EnvirePhysics);

