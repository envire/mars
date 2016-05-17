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

#include "graphviz.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/sim/ConfigMapItem.h>
#include <base/TransformWithCovariance.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <stdlib.h>
#include <algorithm>
#include <cassert>
#include <sstream>

using namespace mars::plugins::graph_viz_plugin;
using namespace mars::utils;
using namespace mars::interfaces;
using namespace envire::core;
using namespace mars::sim;
using namespace std;
using namespace base;
using vertex_descriptor = envire::core::GraphTraits::vertex_descriptor;

//LOG_DEBUG with stringstream for easy conversion
#define LOG_DEBUG_S(...) \
  std::stringstream ss; \
  ss << __VA_ARGS__; \
  LOG_DEBUG(ss.str());


GraphViz::GraphViz(lib_manager::LibManager *theManager)
  : MarsPluginTemplate(theManager, "GraphViz"), GraphEventDispatcher(), originId("")
{

}

void GraphViz::init() 
{
  assert(control->graph != nullptr);
  GraphEventDispatcher::subscribe(control->graph.get());
  GraphItemEventDispatcher<envire::core::Item<envire::smurf::Visual>>::subscribe(control->graph.get());
  GraphItemEventDispatcher<envire::core::Item<smurf::Frame>>::subscribe(control->graph.get());
  GraphItemEventDispatcher<envire::core::Item<smurf::Collidable>>::subscribe(control->graph.get());
  GraphItemEventDispatcher<envire::core::Item<::smurf::Joint>>::subscribe(control->graph.get());
}

void GraphViz::reset() {
}

void GraphViz::frameAdded(const FrameAddedEvent& e)
{
  //use the first frame we get as originId
  if(originId.empty())
  {
    changeOrigin(e.frame);
  }
}


void GraphViz::edgeAdded(const envire::core::EdgeAddedEvent& e)
{
  const vertex_descriptor target = control->graph->vertex(e.target);
  const vertex_descriptor origin = control->graph->vertex(e.origin);
  //update the tree structure
  //find the vertex that was added
  if(tree.find(origin) != tree.end() &&
      tree.find(target) == tree.end())
  {
    //origin already is a parent in the tree and target is not
    tree[origin].children.insert(target);
    tree[target].parent = origin;
  }
  else if(tree.find(origin) == tree.end() &&
          tree.find(target) != tree.end())
  {
    //target already is a parent in the tree and origin is not
    tree[target].children.insert(origin);
    tree[origin].parent = target;
  }
  else
  {
    // This can happen in three cases:
    // 1: A loop was added to the current tree, in this case a new, shorter
    //    path might have been added, therefore we update the whole tree.
    // 2: A transform was added to another sub-tree, we dont care about this.
    // 3:
#define L The current tree is empty and this is the first transform that is added.
    updateTree(originId);
  }
  
}

void GraphViz::edgeRemoved(const envire::core::EdgeRemovedEvent& e)
{
  //Removing a transform can lead to non trivial changes in the tree.
  //Instead of thinking about them we just recalculate the tree.
  //This is fast enough for now.
  updateTree(originId);
}

void GraphViz::edgeModified(const envire::core::EdgeModifiedEvent& e)
{
  const vertex_descriptor target = control->graph->vertex(e.target);
  const vertex_descriptor origin = control->graph->vertex(e.origin);
  vertex_descriptor child;
  
  if(isParent(target, origin))
  {
    child = origin;
  }
  else if(isParent(origin, target))
  {
    child = target;
  }
  else
  {
    //a transform changed that is not part of the current tree. We don't care
    //about it.
    return;
  }
  //update all children
  vector<vertex_descriptor> queue;
  queue.push_back(child);
  while(!queue.empty())
  {
    const vertex_descriptor vertex = queue.back();
    queue.pop_back();
    
    updatePosition<Item<envire::smurf::Visual>>(vertex);
    updatePosition<Item<smurf::Frame>>(vertex);
    
    //if the vertex has children, queue them
    if(tree.find(vertex) != tree.end())
    {
      for(const vertex_descriptor vd : tree[vertex].children)
      {
        queue.emplace_back(vd);
      }
    }
  }
}

void GraphViz::setPos(const envire::core::FrameId& frame, mars::interfaces::NodeData& node)
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

void GraphViz::itemAdded(const envire::core::ItemAddedEvent& e)
{
  //FIXME replace with specific itemAddedEvent for PhysicsConfigMapItem
  boost::shared_ptr<PhysicsConfigMapItem> pItem;
  if(pItem = boost::dynamic_pointer_cast<PhysicsConfigMapItem>(e.item))
  {
    //assert that this item has not been added before
    assert(uuidToGraphicsId.find(pItem->getID()) == uuidToGraphicsId.end());
    try
    {         
      NodeData node;
      if(node.fromConfigMap(&pItem->getData(), ""))
      {
        // TODO Fix: The emission Front is lost when going to config map and back
        node.material.emissionFront = mars::utils::Color(1.0, 1.0, 1.0, 1.0);
        node.material.transparency = 0.5;
        setPos(e.frame, node);
        uuidToGraphicsId[pItem->getID()] = control->graphics->addDrawObject(node);
      }
    }
    catch(const UnknownTransformException& ex)
    {
      LOG_ERROR(ex.what());
    }
  }
}

void GraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<envire::smurf::Visual>>& e)
{
    envire::smurf::Visual vis = e.item->getData();
    addVisual(vis, e.frame, e.item->getID());
    
}

void GraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Collidable>>& e)
{
    if (viewCollidables)
    {
        LOG_DEBUG("Added Collidable");
        smurf::Collidable col = e.item->getData();
        urdf::Collision collision = col.getCollision();
        boost::shared_ptr<urdf::Geometry> geom = collision.geometry;
        switch(geom->type)
        {
            case urdf::Geometry::BOX:
            {
                LOG_DEBUG("BOX");
                //FIXME copy paste code from addBox()
                boost::shared_ptr<urdf::Box> box = boost::dynamic_pointer_cast<urdf::Box>(geom);
                base::Vector3d extents(box->dim.x, box->dim.y, box->dim.z);
                NodeData node;
                node.initPrimitive(mars::interfaces::NODE_TYPE_BOX, extents, 0);
                node.material.transparency = 0.5;
                node.material.emissionFront = mars::utils::Color(0.0, 0.0, 0.8, 1.0);  
                setPos(e.frame, node);
                uuidToGraphicsId[e.item->getID()] = control->graphics->addDrawObject(node); //remeber graphics handle
            }
            break;
            case urdf::Geometry::CYLINDER:
            {
                LOG_DEBUG("CYLINDER");
                //FIXME copy paste code from addCylinder()
                boost::shared_ptr<urdf::Cylinder> cylinder = boost::dynamic_pointer_cast<urdf::Cylinder>(geom);
                //x = length, y = radius, z = not used
                base::Vector3d extents(cylinder->radius, cylinder->length, 0);
                NodeData node;
                node.initPrimitive(mars::interfaces::NODE_TYPE_CYLINDER, extents, 0); //mass is zero because it doesnt matter for visual representation
                node.material.transparency = 0.5;
                node.material.emissionFront = mars::utils::Color(0.0, 0.0, 0.8, 1.0);  
                setPos(e.frame, node); //set link position
                uuidToGraphicsId[e.item->getID()] = control->graphics->addDrawObject(node); //remeber graphics handle
            }
            break;
            case urdf::Geometry::MESH:
                LOG_DEBUG("MESH");
                //addMesh(visual, frameId, uuid);
                break;
            case urdf::Geometry::SPHERE:
            {
                boost::shared_ptr<urdf::Sphere> sphere = boost::dynamic_pointer_cast<urdf::Sphere>(geom);      
                //y and z are unused
                base::Vector3d extents(sphere->radius, 0, 0);
                NodeData node;
                node.initPrimitive(mars::interfaces::NODE_TYPE_SPHERE, extents, 0); //mass is zero because it doesnt matter for visual representation
                node.material.transparency = 0.5;
                node.material.emissionFront = mars::utils::Color(0.0, 0.0, 0.8, 1.0);  
                setPos(e.frame, node); //set link position
                uuidToGraphicsId[e.item->getID()] = control->graphics->addDrawObject(node); //remeber graphics handle
            }
            break;
            default:
                LOG_ERROR("[Envire Graphics] ERROR: unknown geometry type");
        }
    }
}

void GraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Joint>>& e)
{
    if (viewJoints)
    {
        const FrameId source = e.item->getData().getSourceFrame().getName();
        const FrameId target = e.item->getData().getTargetFrame().getName();
        
        
        const envire::core::Transform tf = control->graph->getTransform(source, target);
        const double length = tf.transform.translation.norm();
        base::Vector3d extents(0.01, length, 0);
        
        NodeData node;
        node.initPrimitive(mars::interfaces::NODE_TYPE_CYLINDER, extents, 0); //mass is zero because it doesnt matter for visual representation
        node.material.emissionFront = mars::utils::Color(0.0, 1.0, 0.0, 1.0);    
        node.material.transparency = 0.5;
        
        const envire::core::Transform originToSource = control->graph->getTransform(originId, source); 
        const envire::core::Transform originToTarget = control->graph->getTransform(originId, target); 
        node.pos = (originToSource.transform.translation + originToTarget.transform.translation) / 2.0;
        node.rot = e.item->getData().getParentToJointOrigin().rotation();
        
        uuidToGraphicsId[e.item->getID()] = control->graphics->addDrawObject(node); //remeber graphics handle
    }
}

void GraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Frame>>& e)
{
    if (viewFrames)
    {
        boost::shared_ptr<urdf::Sphere> sphere( new urdf::Sphere);
        sphere->radius = 0.01;
        //y and z are unused
        base::Vector3d extents(sphere->radius, 0, 0);
        //LOG_DEBUG_S("[Envire Graphics] add SPHERE visual. name: " << visual.name << ", frame: "   << frameId << ", radius: " << sphere->radius);
        
        NodeData node;
        node.initPrimitive(mars::interfaces::NODE_TYPE_SPHERE, extents, 0); //mass is zero because it doesnt matter for visual representation
        //setNodeDataMaterial(node, visual.material);
        //node.material.transparency = 0.5;
        node.material.emissionFront = mars::utils::Color(1.0, 0.0, 0.0, 1.0);
        
        setPos(e.frame, node); //set link position
        uuidToGraphicsId[e.item->getID()] = control->graphics->addDrawObject(node); //remeber graphics handle
    }
}

void GraphViz::addVisual(const envire::smurf::Visual& visual, const FrameId& frameId,
                         const boost::uuids::uuid& uuid)
{
  switch(visual.geometry->type)
  {
    case urdf::Geometry::BOX:
      addBox(visual, frameId, uuid);
      break;
    case urdf::Geometry::CYLINDER:
      addCylinder(visual, frameId, uuid);
      break;
    case urdf::Geometry::MESH:
      addMesh(visual, frameId, uuid);
      break;
    case urdf::Geometry::SPHERE:
      addSphere(visual, frameId, uuid);
      break;
    default:
      LOG_ERROR("[Envire Graphics] ERROR: unknown geometry type");
  }
}

void GraphViz::addSphere(const envire::smurf::Visual& visual, const FrameId& frameId, const boost::uuids::uuid& uuid)
{
  boost::shared_ptr<urdf::Sphere> sphere = boost::dynamic_pointer_cast<urdf::Sphere>(visual.geometry);
  assert(sphere.get() != nullptr);
  
  //y and z are unused
  base::Vector3d extents(sphere->radius, 0, 0);
  //LOG_DEBUG_S("[Envire Graphics] add SPHERE visual. name: " << visual.name << ", frame: "   << frameId << ", radius: " << sphere->radius);
  
  NodeData node;
  node.initPrimitive(mars::interfaces::NODE_TYPE_SPHERE, extents, 0); //mass is zero because it doesnt matter for visual representation
  setNodeDataMaterial(node, visual.material);
  
  setPos(frameId, node); //set link position
  uuidToGraphicsId[uuid] = control->graphics->addDrawObject(node); //remeber graphics handle
}


void GraphViz::addBox(const envire::smurf::Visual& visual, const FrameId& frameId, const boost::uuids::uuid& uuid)
{
  boost::shared_ptr<urdf::Box> box = boost::dynamic_pointer_cast<urdf::Box>(visual.geometry);
  assert(box.get() != nullptr);
  
  base::Vector3d extents(box->dim.x, box->dim.y, box->dim.z);
  //LOG_DEBUG_S("[Envire Graphics] add BOX visual. name: " << visual.name << ", frame: "  << frameId << ", size: " << extents.transpose());
  
  NodeData node;
  node.initPrimitive(mars::interfaces::NODE_TYPE_BOX, extents, 0); //mass is zero because it doesnt matter for visual representation
  setNodeDataMaterial(node, visual.material);
  
  setPos(frameId, node); //set link position
  uuidToGraphicsId[uuid] = control->graphics->addDrawObject(node); //remeber graphics handle
}

void GraphViz::addCylinder(const envire::smurf::Visual& visual, const FrameId& frameId, const boost::uuids::uuid& uuid)
{
  boost::shared_ptr<urdf::Cylinder> cylinder = boost::dynamic_pointer_cast<urdf::Cylinder>(visual.geometry);
  assert(cylinder.get() != nullptr);
    
  //x = length, y = radius, z = not used
  base::Vector3d extents(cylinder->radius, cylinder->length, 0);
  
  //LOG_DEBUG_S("[Envire Graphics] add CYLINDER visual. name: " << visual.name << ", frame: "   << frameId << ", radius: " << cylinder->radius << ", length: " << cylinder->length);

  NodeData node;
  node.initPrimitive(mars::interfaces::NODE_TYPE_CYLINDER, extents, 0); //mass is zero because it doesnt matter for visual representation
  setNodeDataMaterial(node, visual.material);
  
  setPos(frameId, node); //set link position
  uuidToGraphicsId[uuid] = control->graphics->addDrawObject(node); //remeber graphics handle
}


void GraphViz::addMesh(const envire::smurf::Visual& visual, const FrameId& frameId, const boost::uuids::uuid& uuid)
{
  boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh>(visual.geometry);
  assert(mesh.get() != nullptr);
  
  //LOG_DEBUG("[Envire Graphics] add MESH visual. name: " + visual.name + ", frame: "  + frameId + ", file: " + mesh->filename);
  
  NodeData node;
  node.init(frameId + "_" + visual.name);
  node.filename = mesh->filename;
  node.physicMode = NodeType::NODE_TYPE_MESH;
  node.visual_scale << mesh->scale.x, mesh->scale.y, mesh->scale.z;
  setNodeDataMaterial(node, visual.material);

  setPos(frameId, node); //set link position
  uuidToGraphicsId[uuid] = control->graphics->addDrawObject(node); //remeber graphics handle
}

void GraphViz::setNodeDataMaterial(NodeData& nodeData, boost::shared_ptr< urdf::Material > material) const
{
  nodeData.material.texturename = material->texture_filename;
  nodeData.material.diffuseFront = mars::utils::Color(material->color.r, material->color.g,
                                                      material->color.b, material->color.a);
}


bool GraphViz::isParent(vertex_descriptor parent, vertex_descriptor child) const
{
  if(tree.find(child) != tree.end())
  {
    bool res = tree.at(child).parent == parent;
    return res;
  }
  return false;
}

void GraphViz::update(sReal time_ms) {
  //uncomment this to randomly change the origin
  /*
  static sReal secondsPassed = 0;
  secondsPassed += time_ms / 1000;
  if(secondsPassed > 500)
  {
    VertexRelationMap::iterator it = tree.begin();
    int randomIndex = rand() % tree.size();
    std::advance(it, randomIndex);
    changeOrigin(control->graph->getFrameId(it->first));
    secondsPassed = 0;
  }
*/
}

void GraphViz::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
}

void GraphViz::changeOrigin(const FrameId& origin)
{
  originId = origin;  
  updateTree(origin);
} 

void GraphViz::updateTree(const FrameId& origin)
{
  const vertex_descriptor newOrigin = control->graph->vertex(origin);
  assert(newOrigin != control->graph->null_vertex());
  tree = control->graph->getTree(newOrigin).tree;
  //update the origins position
  updatePosition<Item<envire::smurf::Visual>>(newOrigin);
  //update all childreen
  for(const auto& it : tree)
  {
    for(vertex_descriptor vertex : it.second.children)
    {
      updatePosition<Item<envire::smurf::Visual>>(vertex);
    }
  }
}


/**Updates the drawing position of @p vertex */              
template <class physicsType> void GraphViz::updatePosition(const vertex_descriptor vertex) const
{
  const FrameId& frameId = control->graph->getFrameId(vertex);
  base::Vector3d translation;
  base::Quaterniond orientation;
   
  if(originId.compare(frameId) == 0)
  {
    translation << 0, 0, 0;
    orientation.setIdentity();
  }
  else
  {
    const Transform tf = control->graph->getTransform(originId, frameId);
    translation = tf.transform.translation;
    orientation = tf.transform.orientation;
  }
  
  using Iterator = EnvireGraph::ItemIterator<physicsType>;
  Iterator begin, end;
  boost::tie(begin, end) = control->graph->getItems<physicsType>(vertex);
  for(;begin != end; ++begin)
  {
    const physicsType& item = *begin;
    //others might use the same types as well, therefore check if if this is one of ours
    if(uuidToGraphicsId.find(item.getID()) != uuidToGraphicsId.end())
    {
      const int graphicsId = uuidToGraphicsId.at(item.getID());
      control->graphics->setDrawObjectPos(graphicsId, translation);
      control->graphics->setDrawObjectRot(graphicsId, orientation);
    }
  }
}

DESTROY_LIB(mars::plugins::graph_viz_plugin::GraphViz);
CREATE_LIB(mars::plugins::graph_viz_plugin::GraphViz);
