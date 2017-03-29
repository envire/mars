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

/**
 * \file EnvireSmurfLoader.h
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Tests
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_SIMNODECREATOR_H
#define MARS_PLUGINS_SIMNODECREATOR_H

#ifdef _PRINT_HEADER_
  #warning "SimNodeCreator.h"
#endif

#include <string>

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/MARSDefs.h>
#include <mars/interfaces/NodeData.h>

#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

#include <mars/sim/PhysicsMapper.h>
#include <mars/sim/SimNode.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphTypes.hpp>
#include <envire_core/items/Transform.hpp>
#include <envire_core/items/Item.hpp>

#include <envire_smurf/Visual.hpp>

#include <smurf/Robot.hpp>

#include <mars/utils/misc.h>
#include <mars/utils/mathUtils.h>

namespace mars {
  namespace plugins {
    namespace EnvireSmurfLoader {

        template <class ItemDataType>
        class SimNodeCreator 
        {
        protected:
            mars::interfaces::ControlCenter *control;
            envire::core::FrameId origin_frame_id;
            std::string type_name;

            virtual mars::interfaces::NodeData createNodeData(const ItemDataType &item_data) = 0;

            void poseToVectorAndQuaternion(const urdf::Pose &pose, mars::utils::Vector *v, mars::utils::Quaternion*q) {
                *v = mars::utils::Vector(pose.position.x, pose.position.y, pose.position.z);
                *q = mars::utils::quaternionFromMembers(pose.rotation); //Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
            }            

        public:
            SimNodeCreator(mars::interfaces::ControlCenter *control, envire::core::FrameId origin_frame_id, std::string type_name)
                : control(control), origin_frame_id(origin_frame_id), type_name(type_name)
            {}

            void create(envire::core::EnvireGraph::vertex_iterator v_itr) 
            {
                envire::core::FrameId frame_id = control->graph->getFrameId(*v_itr);

                using Item = envire::core::Item<ItemDataType>;
                using ItemItr = envire::core::EnvireGraph::ItemIterator<Item>;

                const std::pair<ItemItr, ItemItr> pair = control->graph->getItems<Item>(*v_itr);
#ifdef DEBUG
                if (pair.first == pair.second) {
                    LOG_DEBUG(("[SimNodeCreator::create] No " + type_name + " was found").c_str());
                }
#endif                 
                ItemItr i_itr;
                for(i_itr = pair.first; i_itr != pair.second; i_itr++)
                {
                    const ItemDataType &item_data = i_itr->getData();

#ifdef DEBUG
                    LOG_DEBUG(("[SimNodeCreator::create] " + type_name + " ***" + item_data.getName() + "*** was found" ).c_str());
#endif

                    // create NodeData
                    mars::interfaces::NodeData node_data = createNodeData(item_data);
                    node_data.frameID = frame_id;

                    // set Pose of NodeData according to Frame
                    setPose(node_data, frame_id);

                    // create SimNode with NodeData
                    //createSimNode(node_data, frame_id, item_data.getName()); 
                    control->nodes->addNode(&node_data);
                }         
            }

        private:
            void createSimNode(mars::interfaces::NodeData &node_data, envire::core::FrameId frame_id, std::string name) 
            {
                // create NodePhysik
                mars::interfaces::NodeInterface* node_physics = mars::sim::PhysicsMapper::newNodePhysics(control->sim->getPhysics());

                bool instantiated = false;
                if(node_data.physicMode == mars::interfaces::NODE_TYPE_MLS) {
                    //instantiated = addMlsSurface(node.get());
                    LOG_ERROR("[SimNodeCreator::createSimNode] NOT IMPLEMENTED NODE_TYPE_MLS");
                }
                else  {
                    instantiated = (node_physics->createNode(&node_data));
                }

                // create SimNode and add it into the graph
                if (instantiated) {                           
                    //NOTE Create and store also a simNode. The simNode interface is set to the physics node
                    mars::sim::SimNode *simNode = new mars::sim::SimNode(control, node_data); 
                    simNode->setInterface(node_physics);
                    std::shared_ptr<mars::sim::SimNode> simNodePtr(simNode);

                    simNode->getInterface();

                    using SimNodeItemPtr = envire::core::Item<std::shared_ptr<mars::sim::SimNode>>::Ptr;
                    using SimNodeItem =  envire::core::Item<std::shared_ptr<mars::sim::SimNode>>;

                    SimNodeItemPtr simNodeItem( new SimNodeItem(simNodePtr));        
                    control->graph->addItemToFrame(frame_id, simNodeItem);

                    simNodeItem->getData()->getInterface();
#ifdef DEBUG
                    LOG_DEBUG(("[SimNodeCreator::createSimNode] The SimNode ***" + simNode->getName() + "*** is created for ***" + name + "***").c_str());
#endif                   


                } else {
                    LOG_ERROR(("[SimNodeCreator::createSimNode] Failed to create SimNode for ***" + name + "***").c_str());
                }                  
            }

            void setPose(mars::interfaces::NodeData& node_data, envire::core::FrameId frame_id)
            {
                envire::core::Transform fromOrigin;
                if(origin_frame_id.compare(frame_id) == 0)
                {
                    //this special case happens when the graph only contains one frame
                    //and items are added to that frame. In that case asking the graph 
                    //for the transformation would cause an exception
                    fromOrigin.setTransform(base::TransformWithCovariance::Identity());
                }
                else
                {
                    fromOrigin = control->graph->getTransform(origin_frame_id, frame_id); 
                }
                node_data.pos = fromOrigin.transform.translation;
                node_data.rot = fromOrigin.transform.orientation;
            }
        };

        class SimNodeCreatorFrame: public SimNodeCreator<smurf::Frame>
        {
        public:
            SimNodeCreatorFrame(mars::interfaces::ControlCenter *control, envire::core::FrameId origin_frame_id)
                : SimNodeCreator(control, origin_frame_id, std::string("smurf::Frame"))
            {}            

        private:
            virtual mars::interfaces::NodeData createNodeData(const smurf::Frame &frame)
            {
                // FIX: do we need to add frames into simuation
                // to create simnode for each frame???

                boost::shared_ptr<urdf::Sphere> sphere( new urdf::Sphere);
                sphere->radius = 0.01;
                //y and z are unused
                base::Vector3d extents(sphere->radius, 0, 0);

                // create NodeData
                mars::interfaces::NodeData node;
                node.init(frame.getName());
                node.initPrimitive(mars::interfaces::NODE_TYPE_SPHERE, extents, 0.00001); //mass is zero because it doesnt matter for visual representation
                node.material.emissionFront = mars::utils::Color(1.0, 0.0, 0.0, 1.0);
                node.c_params.coll_bitmask = 0;
                node.movable = true;
                node.groupID = frame.getGroupId();
                node.density = 0.0;      

                // FIX: show the frame as small spheres
                // change to physical representation for example

                // add empty visual
                node.map["origname"] = "";
                node.map["materialName"] = "_emptyVisualMaterial";
                node.map["visualType"] = "empty";    

                node.simNodeType = mars::interfaces::SimNodeType::FRAME;                       
                
                return node; 
            }            
        };

        class SimNodeCreatorCollidable: public SimNodeCreator<smurf::Collidable>
        {
        public:
            SimNodeCreatorCollidable(mars::interfaces::ControlCenter *control, envire::core::FrameId origin_frame_id)
                : SimNodeCreator(control, origin_frame_id, std::string("smurf::Collidable"))
            {}            

        private:         
            virtual mars::interfaces::NodeData createNodeData(const smurf::Collidable &collidable)
            {
                urdf::Collision collision = collidable.getCollision(); 

                configmaps::ConfigMap config;
                std::string name;
                if (collision.name.empty()) {
                    name = "collision_"; // FIXME
                } else {
                    name = collision.name;
                }

                // init node
                config["name"] = name;
                //config["index"] = nextNodeID++;
                config["groupid"] = collidable.getGroupId(); 
                config["movable"] = true ;//config["movable"] = !fixed;
                //config["relativeid"] = currentNodeID;
                config["mass"] = 0.001;
                config["density"] = 0.0;

                // parse geometry
                urdf::GeometrySharedPtr tmpGeometry = collision.geometry;
                mars::utils::Vector size(0.0, 0.0, 0.0);
                mars::utils::Vector scale(1.0, 1.0, 1.0);
                urdf::Vector3 tmpV;
                switch (tmpGeometry->type) {
                    case urdf::Geometry::SPHERE:
                        size.x() = ((urdf::Sphere*) tmpGeometry.get())->radius;
                        config["physicmode"] = "sphere";
                        break;
                    case urdf::Geometry::BOX:
                        tmpV = ((urdf::Box*) tmpGeometry.get())->dim;
                        size = mars::utils::Vector(tmpV.x, tmpV.y, tmpV.z);
                        config["physicmode"] = "box";
                        break;
                    case urdf::Geometry::CYLINDER:
                        size.x() = ((urdf::Cylinder*) tmpGeometry.get())->radius;
                        size.y() = ((urdf::Cylinder*) tmpGeometry.get())->length;
                        config["physicmode"] = "cylinder";
                        break;
                    case urdf::Geometry::MESH:
                        tmpV = ((urdf::Mesh*) tmpGeometry.get())->scale;
                        scale = mars::utils::Vector(tmpV.x, tmpV.y, tmpV.z);
                        mars::utils::vectorToConfigItem(&config["physicalScale"][0], &scale);
                        config["filename"] = ((urdf::Mesh*) tmpGeometry.get())->filename;
                        config["origname"] = "";
                        config["physicmode"] = "mesh";
                        config["loadSizeFromMesh"] = true;
                        break;
                    default:
                        config["physicmode"] = "undefined";
                        break;
                }
                mars::utils::vectorToConfigItem(&config["extend"][0], &size);
                mars::utils::vectorToConfigItem(&config["scale"][0], &scale);
                // FIXME: We need to correctly deal with scale and size in MARS
                //       if we have a mesh here, as a first hack we use the scale as size

                // pose
                mars::utils::Vector v;
                mars::utils::Quaternion q;
                poseToVectorAndQuaternion(collision.origin, &v, &q);
                mars::utils::vectorToConfigItem(&config["position"][0], &v);
                mars::utils::quaternionToConfigItem(&config["rotation"][0], &q);

                // addEmptyVisualToNode(&config);
                config["origname"] = "";
                config["materialName"] = "_emptyVisualMaterial";
                config["visualType"] = "empty";     

                mars::interfaces::NodeData node;  
                node.fromConfigMap(&config, "", control->loadCenter);
                node.simNodeType = mars::interfaces::SimNodeType::COLLISION;
                
                return node; 
            }      
        };

        class SimNodeCreatorInertial: public SimNodeCreator<smurf::Inertial>
        {
        public:
            SimNodeCreatorInertial(mars::interfaces::ControlCenter *control, envire::core::FrameId origin_frame_id)
                : SimNodeCreator(control, origin_frame_id, std::string("smurf::Inertial"))
            {}    

        private:
            virtual mars::interfaces::NodeData createNodeData(const smurf::Inertial &inertial)
            {
                urdf::Inertial inertial_urd = inertial.getUrdfInertial();

                configmaps::ConfigMap config;
                std::string name;
                if (inertial.getName().empty()) {
                    name = "inertial_"; // FIXME
                } else {
                    name = "inertial_" + inertial.getName();
                }

                // init node
                config["name"] = name;
                //config["index"] = nextNodeID++;
                config["groupid"] = inertial.getGroupId();
                config["movable"] = true;
                //config["relativeid"] = currentNodeID;

                // add inertial information

                config["density"] = 0.0;
                config["mass"] = inertial_urd.mass;
                config["inertia"] = true;
                config["i00"] = inertial_urd.ixx;
                config["i01"] = inertial_urd.ixy;
                config["i02"] = inertial_urd.ixz;
                config["i10"] = inertial_urd.ixy;
                config["i11"] = inertial_urd.iyy;
                config["i12"] = inertial_urd.iyz;
                config["i20"] = inertial_urd.ixz;
                config["i21"] = inertial_urd.iyz;
                config["i22"] = inertial_urd.izz;

                // pose
                mars::utils::Vector v;
                mars::utils::Quaternion q;
                urdf::Pose pose;
                pose = inertial_urd.origin;
                poseToVectorAndQuaternion(pose, &v, &q);
                mars::utils::vectorToConfigItem(&config["position"][0], &v);
                mars::utils::quaternionToConfigItem(&config["rotation"][0], &q);

                // complete node
                //addEmptyVisualToNode(&config);
                config["origname"] = "";
                config["materialName"] = "_emptyVisualMaterial";
                config["visualType"] = "empty";
                //addEmptyCollisionToNode(&config);
                mars::utils::Vector size(0.01, 0.01, 0.01);
                config["physicmode"] = "box";
                config["coll_bitmask"] = 0;                

                mars::interfaces::NodeData node;  
                node.fromConfigMap(&config, "", control->loadCenter);
                node.simNodeType = mars::interfaces::SimNodeType::INERTIA;                
                
                return node; 
            }               
        };

        class SimNodeCreatorVisual: public SimNodeCreator<envire::smurf::Visual>
        {
        public:
            SimNodeCreatorVisual(mars::interfaces::ControlCenter *control, envire::core::FrameId origin_frame_id)
                : SimNodeCreator(control, origin_frame_id, std::string("envire::smurf::Visual"))
            {}    

        private:

            virtual mars::interfaces::NodeData createNodeData(const envire::smurf::Visual &visual)
            {
                configmaps::ConfigMap config;
                std::string name;
                if (visual.name.empty()) {
                    name = "visual_"; // FIXME
                } else {
                    name = visual.name;
                }

                // init node
                config["name"] = name;
                config["groupid"] = visual.groupId;          // FIX: set right group id
                config["movable"] = true;     // FIX
                //config["relativeid"] = currentNodeID
                config["mass"] = 0.001;
                config["density"] = 0.0;
                mars::utils::Vector v(0.001, 0.001, 0.001);
                mars::utils::vectorToConfigItem(&config["extend"][0], &v);

                  // parse position
                mars::utils::Quaternion q;
                poseToVectorAndQuaternion(visual.origin, &v, &q);
                mars::utils::vectorToConfigItem(&config["position"][0], &v);
                mars::utils::quaternionToConfigItem(&config["rotation"][0], &q);

                  // parse geometry
                urdf::GeometrySharedPtr tmpGeometry = visual.geometry;
                mars::utils::Vector size(0.0, 0.0, 0.0);
                mars::utils::Vector scale(1.0, 1.0, 1.0);
                urdf::Vector3 tmpV;
                config["filename"] = "PRIMITIVE";
                switch (tmpGeometry->type) {
                  case urdf::Geometry::SPHERE:
                    size.x() = 2.0*((urdf::Sphere*) tmpGeometry.get())->radius;
                    size.y() = 2.0*((urdf::Sphere*) tmpGeometry.get())->radius;
                    size.z() = 2.0*((urdf::Sphere*) tmpGeometry.get())->radius;
                    config["origname"] = "sphere";
                    break;
                  case urdf::Geometry::BOX:
                    tmpV = ((urdf::Box*) tmpGeometry.get())->dim;
                    size = mars::utils::Vector(tmpV.x, tmpV.y, tmpV.z);
                    config["origname"] = "box";
                    break;
                  case urdf::Geometry::CYLINDER:
                    size.x() = 2.0*((urdf::Cylinder*) tmpGeometry.get())->radius;
                    size.y() = 2.0*((urdf::Cylinder*) tmpGeometry.get())->radius;
                    size.z() = ((urdf::Cylinder*) tmpGeometry.get())->length;
                    config["origname"] = "cylinder";
                    break;
                  case urdf::Geometry::MESH:
                    tmpV = ((urdf::Mesh*) tmpGeometry.get())->scale;
                    scale = mars::utils::Vector(tmpV.x, tmpV.y, tmpV.z);
                    config["filename"] = ((urdf::Mesh*) tmpGeometry.get())->filename;
                    config["origname"] = "";
                    break;
                  default:
                    break;
                  }
                mars::utils::vectorToConfigItem(&config["visualsize"][0], &size);
                mars::utils::vectorToConfigItem(&config["visualscale"][0], &scale);
                config["materialName"] = visual.material_name;

                //addEmptyCollisionToNode(&config);
                mars::utils::Vector size_tmp(0.01, 0.01, 0.01);
                config["physicmode"] = "box";
                config["coll_bitmask"] = 0;
                mars::utils::vectorToConfigItem(&config["extend"][0], &size_tmp);  

                mars::interfaces::NodeData node;  
                node.fromConfigMap(&config, "", control->loadCenter);

                // ---- create materials
                configmaps::ConfigMap material_config;
                //material_config["id"] = nextMaterialID++;
                material_config["name"] = visual.material->name;
                material_config["exists"] = true;
                material_config["diffuseFront"][0]["a"] = (double) visual.material->color.a;
                material_config["diffuseFront"][0]["r"] = (double) visual.material->color.r;
                material_config["diffuseFront"][0]["g"] = (double) visual.material->color.g;
                material_config["diffuseFront"][0]["b"] = (double) visual.material->color.b;
                material_config["texturename"] = visual.material->texture_filename;

                // FIX: tmpPath
                std::string tmpPath("");

                mars::interfaces::MaterialData material;
                int valid = material.fromConfigMap(&material_config, tmpPath);
                node.material = material;
                node.simNodeType = mars::interfaces::SimNodeType::VISUAL;  

                return node;
            }               
        };        

    } // end of namespace EnvireSmurfLoader
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_ENVIRESMURFLOADER
