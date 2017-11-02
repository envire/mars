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

#include "EnvireGraphViz.h"
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
#include <mars/sim/defines.hpp>

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

EnvireGraphViz::EnvireGraphViz(lib_manager::LibManager *theManager)
  : MarsPluginTemplate(theManager, "EnvireGraphViz"), GraphEventDispatcher(), originId(SIM_CENTER_FRAME_NAME)
{
    // FIX: take originId from the node manager
    // change the name of origin id from center into the world
    updateTree(originId);
}

void EnvireGraphViz::init() 
{
  assert(control->graph != nullptr);
  GraphEventDispatcher::subscribe(control->graph.get());
  //GraphItemEventDispatcher<envire::core::Item<smurf::Visual>>::subscribe(control->graph.get());
  //GraphItemEventDispatcher<envire::core::Item<smurf::Frame>>::subscribe(control->graph.get());
  //GraphItemEventDispatcher<envire::core::Item<smurf::Collidable>>::subscribe(control->graph.get());
  GraphItemEventDispatcher<envire::core::Item<::smurf::Joint>>::subscribe(control->graph.get());

  GraphItemEventDispatcher<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>::subscribe(control->graph.get());
  GraphItemEventDispatcher<envire::core::Item<maps::grid::MLSMapKalman>>::subscribe(control->graph.get());
  GraphItemEventDispatcher<envire::core::Item<maps::grid::MLSMapPrecalculated>>::subscribe(control->graph.get());
  if(originId.empty())
  {
    LOG_WARN("[EnvireGraphViz::init] Frame center will be added as origin because no origin frame exists");
    envire::core::FrameId center = SIM_CENTER_FRAME_NAME; 
    if (! control->graph->containsFrame(center))
    {
      control->graph->addFrame(center);
    }
    changeOrigin(center);
  }
}

void EnvireGraphViz::reset() {
}

void EnvireGraphViz::frameAdded(const FrameAddedEvent& e)
{
}


void EnvireGraphViz::setPos(const envire::core::FrameId& frame, mars::interfaces::NodeData& node)
{
    Transform fromOrigin;
    if(originId.compare(frame) == 0)
    {
      //this special case happens when the graph only contains one frame
      //and items are added to that frame. In that case asking the graph 
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

void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>& e)
{

    // FIX: do we need? control->sim->sceneHasChanged(false); 
    LOG_DEBUG("[EnvireGraphViz::itemAdded] SimNode is added into the graph ");

    std::shared_ptr<mars::sim::SimNode> simNode = e.item->getData();

    int visual_rep = simNode->getVisualRep(); 

    // ----- Set Visual Representation
    mars::interfaces::NodeData nodeData = simNode->getSNode();
    std::cout << "[EnvireGraphViz::itemAdded] Name: " << nodeData.name << " "
                << nodeData.pos.x() << " " << nodeData.pos.y() << " " << nodeData.pos.z() << std::endl;

    mars::interfaces::NodeId id = control->graphics->addDrawObject(nodeData, visual_rep & 1);
    if(id) {
        std::cout << "setGraphicsID 1 " << std::endl;
        simNode->setGraphicsID(id);
        uuidToGraphicsId[e.item->getID()] = id;
    }    

    /// ---- Set Physical Representation -> collision objects
    if(nodeData.noPhysical == false && nodeData.physicMode != mars::interfaces::NODE_TYPE_TERRAIN) {
        mars::interfaces::NodeData physicalRep;
        physicalRep = nodeData;
        physicalRep.material = nodeData.material;
        physicalRep.material.exists = 1;
        physicalRep.material.transparency = 0.3;
        physicalRep.material.name += "_trans";
        physicalRep.visual_offset_pos = mars::utils::Vector(0.0, 0.0, 0.0);
        physicalRep.visual_offset_rot = mars::utils::Quaternion::Identity();
        physicalRep.visual_size = mars::utils::Vector(0.0, 0.0, 0.0);
        physicalRep.map["sharedDrawID"] = 0lu;
        physicalRep.map["visualType"] = mars::interfaces::NodeData::toString(nodeData.physicMode);                    

        if(nodeData.physicMode != mars::interfaces::NODE_TYPE_MESH) {
            physicalRep.filename = "PRIMITIVE";
            //physicalRep.filename = nodeS->filename;
            if(nodeData.physicMode > 0 && nodeData.physicMode < mars::interfaces::NUMBER_OF_NODE_TYPES){
                physicalRep.origName = mars::interfaces::NodeData::toString(nodeData.physicMode);
            }
        }
            
        id = control->graphics->addDrawObject(physicalRep, visual_rep & 2);
        if(id) {
            std::cout << "setGraphicsID 2 " << std::endl;
            simNode->setGraphicsID2(id);
            uuidToGraphicsId2[e.item->getID()] = id;
        }
    }  
}

void EnvireGraphViz::itemAdded(const envire::core::ItemAddedEvent& e)
{
  //FIXME replace with specific itemAddedEvent for PhysicsConfigMapItem
  boost::shared_ptr<PhysicsConfigMapItem> pItem;
  if(pItem = boost::dynamic_pointer_cast<PhysicsConfigMapItem>(e.item))
  {
    NodeData node1;
    node1.fromConfigMap(&pItem->getData(), "");

  // MLS objects are visualized using their own osg visual (see void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<maps::grid::MLSMapKalman>>& e))
  if(node1.physicMode != NODE_TYPE_MLS) //TODO: implement a visualization for mls
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
}

void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Visual>>& e)
{
    smurf::Visual vis = e.item->getData();
    addVisual(vis, e.frame, e.item->getID());
}

void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Collidable>>& e)
{
    if (viewCollidables)
    {
        LOG_DEBUG("Added Collidable");
        smurf::Collidable col = e.item->getData();
        urdf::Collision collision = col.getCollision();
        urdf::GeometrySharedPtr geom = collision.geometry;
        switch(geom->type)
        {
            case urdf::Geometry::BOX:
            {
                LOG_DEBUG("BOX");
                //FIXME copy paste code from addBox()
                urdf::BoxSharedPtr box = urdf::dynamic_pointer_cast<urdf::Box>(geom);
                base::Vector3d extents(box->dim.x, box->dim.y, box->dim.z);
                NodeData node;
                node.initPrimitive(mars::interfaces::NODE_TYPE_BOX, extents, 0.00001);
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
                urdf::CylinderSharedPtr cylinder = urdf::dynamic_pointer_cast<urdf::Cylinder>(geom);
                //x = length, y = radius, z = not used
                base::Vector3d extents(cylinder->radius, cylinder->length, 0);
                NodeData node;
                node.initPrimitive(mars::interfaces::NODE_TYPE_CYLINDER, extents, 0.00001); //mass is zero because it doesnt matter for visual representation
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
                urdf::SphereSharedPtr sphere = urdf::dynamic_pointer_cast<urdf::Sphere>(geom);
                //y and z are unused
                base::Vector3d extents(sphere->radius, 0, 0);
                NodeData node;
                node.initPrimitive(mars::interfaces::NODE_TYPE_SPHERE, extents, 0.00001); //mass is zero because it doesnt matter for visual representation
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

void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Joint>>& e)
{
    if (viewJoints)
    {
        const FrameId source = e.item->getData().getSourceFrame().getName();
        const FrameId target = e.item->getData().getTargetFrame().getName();
        
        const envire::core::Transform tf = control->graph->getTransform(source, target);
        const double length = tf.transform.translation.norm();
        base::Vector3d extents(0.01, length, 0);
        
        NodeData node;
        node.initPrimitive(mars::interfaces::NODE_TYPE_CYLINDER, extents, 0.00001); //mass is zero because it doesnt matter for visual representation
        node.material.emissionFront = mars::utils::Color(0.0, 1.0, 0.0, 1.0);    
        node.material.transparency = 0.5;
        
        const envire::core::Transform originToSource = control->graph->getTransform(originId, source); 
        const envire::core::Transform originToTarget = control->graph->getTransform(originId, target); 
        node.pos = (originToSource.transform.translation + originToTarget.transform.translation) / 2.0;
        node.rot = e.item->getData().getParentToJointOrigin().rotation();
        
        uuidToGraphicsId[e.item->getID()] = control->graphics->addDrawObject(node); //remeber graphics handle
    }
}

void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Frame>>& e)
{
    if (viewFrames)
    {
        urdf::SphereSharedPtr sphere( new urdf::Sphere);
        sphere->radius = 0.01;
        //y and z are unused
        base::Vector3d extents(sphere->radius, 0, 0);
        //LOG_DEBUG_S("[Envire Graphics] add SPHERE visual. name: " << visual.name << ", frame: "   << frameId << ", radius: " << sphere->radius);
        
        NodeData node;
        node.initPrimitive(mars::interfaces::NODE_TYPE_SPHERE, extents, 0.00001); //mass is zero because it doesnt matter for visual representation
        //setNodeDataMaterial(node, visual.material);
        //node.material.transparency = 0.5;
        node.material.emissionFront = mars::utils::Color(1.0, 0.0, 0.0, 1.0);
        
        setPos(e.frame, node); //set link position
        uuidToGraphicsId[e.item->getID()] = control->graphics->addDrawObject(node); //remeber graphics handle
    }
}

void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<maps::grid::MLSMapKalman>>& e)
{
  LOG_DEBUG("[EnvireGraphViz::itemAdded<MLSMapKalman>] Added an MLS to the graph, let's visualize it");
  maps::grid::MLSMapKalman map = e.item->getData();
  osgNode = createMainNode(); // vizkit3d Protected
  updateData(map);
  updateMainNode(osgNode);// vizkit3d Protected
  osgGroup = getVizNode();
  if (!osgGroup){ LOG_DEBUG("[EnvireGraphViz::itemAdded<MLSMapKalman>] The generated osgGroup is null");}
  else {LOG_DEBUG("[EnvireGraphViz::itemAdded<MLSMapKalman>] The OSG group is not null");}
  control->graphics->addOSGNode(osgNode);
  // We don't get any id back from addOSGNode, so I guess we don't need the following:
  //uuidToGraphicsId[e.item->getID()] = control->graphics->addDrawObject(node); //remeber graphics handle
}

void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<maps::grid::MLSMapPrecalculated>>& e)
{
  LOG_DEBUG("[EnvireGraphViz::itemAdded<MLSMapPrecalculated>] Added an MLS to the graph, let's visualize it");
  maps::grid::MLSMapPrecalculated map = e.item->getData();
  osgNode = createMainNode(); // vizkit3d Protected
  updateData(map);
  updateMainNode(osgNode);// vizkit3d Protected
  osgGroup = getVizNode();
  mlsFrameName = e.frame;
  visTf = new osg::PositionAttitudeTransform();
  visTf->addChild(osgNode);
  updateMLSVis();

  if (!osgGroup){ LOG_DEBUG("[EnvireGraphViz::itemAdded<MLSMapPrecalculated>] The generated osgGroup is null");}
  else {LOG_DEBUG("[EnvireGraphViz::itemAdded<MLSMapPrecalculated>] The OSG group is not null");}
  control->graphics->addOSGNode(visTf);
  // We don't get any id back from addOSGNode, so I guess we don't need the following:
  //uuidToGraphicsId[e.item->getID()] = control->graphics->addDrawObject(node); //remeber graphics handle
}




void EnvireGraphViz::addVisual(const smurf::Visual& visual, const FrameId& frameId,
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

void EnvireGraphViz::addSphere(const smurf::Visual& visual, const FrameId& frameId, const boost::uuids::uuid& uuid)
{
  urdf::SphereSharedPtr sphere = urdf::dynamic_pointer_cast<urdf::Sphere>(visual.geometry);
  assert(sphere.get() != nullptr);
  
  //y and z are unused
  base::Vector3d extents(sphere->radius, 0, 0);
  //LOG_DEBUG_S("[Envire Graphics] add SPHERE visual. name: " << visual.name << ", frame: "   << frameId << ", radius: " << sphere->radius);
  
  NodeData node;
  node.initPrimitive(mars::interfaces::NODE_TYPE_SPHERE, extents, 0.00001); //mass is zero because it doesnt matter for visual representation
  setNodeDataMaterial(node, visual.material);
  
  setPos(frameId, node); //set link position
  uuidToGraphicsId[uuid] = control->graphics->addDrawObject(node); //remeber graphics handle
}


void EnvireGraphViz::addBox(const smurf::Visual& visual, const FrameId& frameId, const boost::uuids::uuid& uuid)
{
  urdf::BoxSharedPtr box = urdf::dynamic_pointer_cast<urdf::Box>(visual.geometry);
  assert(box.get() != nullptr);
  
  base::Vector3d extents(box->dim.x, box->dim.y, box->dim.z);
  //LOG_DEBUG_S("[Envire Graphics] add BOX visual. name: " << visual.name << ", frame: "  << frameId << ", size: " << extents.transpose());
  
  NodeData node;
  node.initPrimitive(mars::interfaces::NODE_TYPE_BOX, extents, 0.00001); //mass is zero because it doesnt matter for visual representation
  setNodeDataMaterial(node, visual.material);
  
  setPos(frameId, node); //set link position
  uuidToGraphicsId[uuid] = control->graphics->addDrawObject(node); //remeber graphics handle
}

void EnvireGraphViz::addCylinder(const smurf::Visual& visual, const FrameId& frameId, const boost::uuids::uuid& uuid)
{
  urdf::CylinderSharedPtr cylinder = urdf::dynamic_pointer_cast<urdf::Cylinder>(visual.geometry);
  assert(cylinder.get() != nullptr);
    
  //x = length, y = radius, z = not used
  base::Vector3d extents(cylinder->radius, cylinder->length, 0);
  
  //LOG_DEBUG_S("[Envire Graphics] add CYLINDER visual. name: " << visual.name << ", frame: "   << frameId << ", radius: " << cylinder->radius << ", length: " << cylinder->length);

  NodeData node;
  node.initPrimitive(mars::interfaces::NODE_TYPE_CYLINDER, extents, 0.00001); //mass is zero because it doesnt matter for visual representation
  setNodeDataMaterial(node, visual.material);
  
  setPos(frameId, node); //set link position
  uuidToGraphicsId[uuid] = control->graphics->addDrawObject(node); //remeber graphics handle
}


void EnvireGraphViz::addMesh(const smurf::Visual& visual, const FrameId& frameId, const boost::uuids::uuid& uuid)
{
  urdf::MeshSharedPtr mesh = urdf::dynamic_pointer_cast<urdf::Mesh>(visual.geometry);
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

void EnvireGraphViz::setNodeDataMaterial(NodeData& nodeData, urdf::MaterialSharedPtr material) const
{
  nodeData.material.texturename = material->texture_filename;
  nodeData.material.diffuseFront = mars::utils::Color(material->color.r, material->color.g,
                                                      material->color.b, material->color.a);
}


void EnvireGraphViz::update(sReal time_ms) {
  const float timeBetweenFramesMs = 1000.0 / visualUpdateRateFps;
  timeSinceLastUpdateMs += time_ms;
  
  if(timeSinceLastUpdateMs >= timeBetweenFramesMs)
  {
    updateVisuals();
    timeSinceLastUpdateMs = 0;
  }
}

void EnvireGraphViz::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
}

void EnvireGraphViz::changeOrigin(const FrameId& origin)
{
  originId = origin;  
  updateTree(origin);
} 

void EnvireGraphViz::updateTree(const FrameId& origin)
{
  const vertex_descriptor newOrigin = control->graph->vertex(origin);
  assert(newOrigin != control->graph->null_vertex());
  tree.clear();
  control->graph->getTree(newOrigin, true, &tree);
}

void EnvireGraphViz::updateMLSVis()
{
  envire::core::Transform simTf = control->graph->getTransform(SIM_CENTER_FRAME_NAME, mlsFrameName);
  osg::Vec3d vector;
  osg::Quat quat;

  vector.x() = simTf.transform.translation.x();
  vector.y() = simTf.transform.translation.y();
  vector.z() = simTf.transform.translation.z();

  quat.x() = simTf.transform.orientation.x();
  quat.y() = simTf.transform.orientation.y();
  quat.z() = simTf.transform.orientation.z();
  quat.w() = simTf.transform.orientation.w();

  visTf->setPosition(vector);
  visTf->setAttitude(quat);
}

void EnvireGraphViz::updateVisuals()
{
  if (tree.hasRoot() == false)
    return;

  tree.visitBfs(tree.root, [&](GraphTraits::vertex_descriptor vd, 
                               GraphTraits::vertex_descriptor parent)
  {
    //updatePosition<Item<smurf::Visual>>(vd);
    //updatePosition<Item<smurf::Frame>>(vd);
    updatePosition<Item<std::shared_ptr<mars::sim::SimNode>>>(vd);
    // Check that the frame exists
    if (control->graph->containsFrame(mlsFrameName))
    {
      updateMLSVis();
    }
  });
}


/**Updates the drawing position of @p vertex */              
template <class physicsType> void EnvireGraphViz::updatePosition(const vertex_descriptor vertex)
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
    if(pathsFromOrigin.find(vertex) == pathsFromOrigin.end())
    {
      //this is an unknown vertex, find the path and store it
      pathsFromOrigin[vertex] = control->graph->getPath(originId, frameId, true);
    }
    const Transform tf = control->graph->getTransform(pathsFromOrigin[vertex]);
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

    if(uuidToGraphicsId2.find(item.getID()) != uuidToGraphicsId2.end())
    {   

      const int graphicsId2 = uuidToGraphicsId2.at(item.getID());
      control->graphics->setDrawObjectPos(graphicsId2, translation);
      control->graphics->setDrawObjectRot(graphicsId2, orientation);       
    }
  }
}

DESTROY_LIB(mars::plugins::graph_viz_plugin::EnvireGraphViz);
CREATE_LIB(mars::plugins::graph_viz_plugin::EnvireGraphViz);
