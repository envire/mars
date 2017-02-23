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
 * \file EnvireSmurfLoader.cpp
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Tests
 *
 * Version 0.1
 */

#include "EnvireSmurfLoader.h"

#include <mars/interfaces/Logging.hpp>

#include <lib_manager/LibManager.hpp>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/LoadCenter.h>
#include <mars/sim/PhysicsMapper.h>
#include <mars/sim/SimNode.h>

#include <lib_config/YAMLConfiguration.hpp>
// To populate the Graph from the smurf

#include <envire_smurf/GraphLoader.hpp>

// For the floor
#include <mars/sim/ConfigMapItem.h>

// To log the graph
#include <base/Time.hpp>
#include <envire_core/graph/GraphViz.hpp>

using vertex_descriptor = envire::core::GraphTraits::vertex_descriptor;

#define DEBUG

namespace mars {
    namespace plugins {
        namespace EnvireSmurfLoader {
            
            using namespace mars::utils;
            using namespace mars::interfaces;
            
            EnvireSmurfLoader::EnvireSmurfLoader(lib_manager::LibManager *theManager)
                : LoadSceneInterface(theManager), control(NULL), nextGroupId(1)
            {
                mars::interfaces::SimulatorInterface *marsSim;
                marsSim = libManager->getLibraryAs<mars::interfaces::SimulatorInterface>("mars_sim");
                if(marsSim) {
                    control = marsSim->getControlCenter();
                    control->loadCenter->loadScene[".zsmurf"] = this; // zipped smurf model
                    control->loadCenter->loadScene[".zsmurfs"] = this; // zipped smurf scene
                    control->loadCenter->loadScene[".smurf"] = this; // smurf model
                    control->loadCenter->loadScene[".smurfs"] = this; // smurf scene
                    control->loadCenter->loadScene[".svg"] = this; // smurfed vector graphic
                    control->loadCenter->loadScene[".urdf"] = this; // urdf model
                    LOG_INFO("envire_smurf_loader: SMURF loader to loadCenter");                    
                }
            }

            EnvireSmurfLoader::~EnvireSmurfLoader() {
              if(control) {
                control->loadCenter->loadScene.erase(".zsmurf");
                control->loadCenter->loadScene.erase(".zsmurfs");
                control->loadCenter->loadScene.erase(".smurf");
                control->loadCenter->loadScene.erase(".smurfs");
                control->loadCenter->loadScene.erase(".svg");
                control->loadCenter->loadScene.erase(".urdf");
                libManager->releaseLibrary("mars_sim");
              }                
            }       

            bool EnvireSmurfLoader::loadFile(std::string filename, std::string tmpPath,
                                    std::string robotname)
            {
                std::cout << "smurf loader zero position" << std::endl;
                vertex_descriptor center = addCenter();
                envire::core::Transform iniPose;
                iniPose.transform.orientation = base::Quaterniond::Identity();
                iniPose.transform.translation << 0.0, 0.0, 0.3;
                addRobot(filename, center, iniPose);
                createSimObjects();
                return true;
            }

            bool EnvireSmurfLoader::loadFile(std::string filename, std::string tmpPath,
                                std::string robotname, utils::Vector pos, utils::Vector rot)
            {
                std::cout << "smurf loader given position" << std::endl;
                vertex_descriptor center = addCenter();
                envire::core::Transform iniPose;
                iniPose.transform.orientation = base::Quaterniond::Identity();
                iniPose.transform.translation << pos.x(), pos.y(), pos.z();
                addRobot(filename, center, iniPose);
                createSimObjects();
            }    

            int EnvireSmurfLoader::saveFile(std::string filename, std::string tmpPath)
            {
                return 0;
            }


            void EnvireSmurfLoader::addRobot(std::string filename, vertex_descriptor center, envire::core::Transform iniPose)
            {
                std::string path = libConfig::YAMLConfigParser::applyStringVariableInsertions(filename); 
                LOG_DEBUG("Robot Path: %s",  path.c_str() );
                smurf::Robot* robot = new( smurf::Robot);
                robot->loadFromSmurf(path);
                envire::smurf::GraphLoader graphLoader(control->graph);
                graphLoader.loadRobot(nextGroupId, center, iniPose, *robot);
            }
            
            vertex_descriptor EnvireSmurfLoader::addCenter()
            {
                center = "center";
                control->graph->addFrame(center);
                return control->graph->getVertex(center);
            }

            void EnvireSmurfLoader::addFloor(const vertex_descriptor &center)
            {
                NodeData data;
                data.init("floorData", Vector(0,0,0));
                data.initPrimitive(interfaces::NODE_TYPE_BOX, Vector(25, 25, 0.1), 0.0001);
                data.movable = false;
                mars::sim::PhysicsConfigMapItem::Ptr item(new mars::sim::PhysicsConfigMapItem);
                data.material.transparency = 0.5;
                //data.material.ambientFront = mars::utils::Color(0.0, 1.0, 0.0, 1.0);
                // TODO Fix the material data is lost in the conversion from/to configmap
                data.material.emissionFront = mars::utils::Color(1.0, 1.0, 1.0, 1.0);
                LOG_DEBUG("Color of the Item in the addFloor: %f , %f, %f, %f", data.material.emissionFront.a , data.material.emissionFront.b, data.material.emissionFront.g, data.material.emissionFront.r );
                data.toConfigMap(&(item.get()->getData()));
                control->graph->addItemToFrame(control->graph->getFrameId(center), item);
            }           

            void EnvireSmurfLoader::createSimObjects()
            {
                //materials
                //nodes

                loadNodes();
                //loadJoints();
                //motors
                //TODO: control->motors->connectMimics();
                //sensors
                //controller
                //light
                //graphic
            }             

            void EnvireSmurfLoader::loadNodes()
            {
#ifdef DEBUG
                LOG_DEBUG("[EnvireSmurfLoader::loadNodes] ------------------- Parse the graph and create SimNodes -------------------");
#endif                
                // search the graph
                envire::core::EnvireGraph::vertex_iterator v_itr, v_end;
                boost::tie(v_itr, v_end) = control->graph->getVertices();
                for(; v_itr != v_end; v_itr++)
                {
#ifdef DEBUG                    
                    envire::core::FrameId frame_id = control->graph->getFrameId(*v_itr);
                    LOG_DEBUG(("[EnvireSmurfLoader::loadNodes] --- IN ***" + frame_id + "*** ---" ).c_str());

#endif 
                    // search for smurf::Frame, Collidable and Inertial
                    loadNode<smurf::Frame>      (v_itr, &createNodeDataForFrame,        "smurf::Frame");
                    loadNode<smurf::Collidable> (v_itr, &createNodeDataForCollidable,   "smurf::Collidable");
                    loadNode<smurf::Inertial>   (v_itr, &createNodeDataForInertial,     "smurf::Inertial");

                    //using FrameItem = envire::core::Item<smurf::Frame>;
                    //if (control->graph->containsItems<FrameItem>(*begin)) {
                    //    smurf::Frame link = e.item->getData()
                }
            }                   

            template <class ItemDataType>
            void EnvireSmurfLoader::loadNode(envire::core::EnvireGraph::vertex_iterator v_itr, 
                mars::interfaces::NodeData (*createNodeData)(const ItemDataType &item_data),
                std::string type_name)
            {
                envire::core::FrameId frame_id = control->graph->getFrameId(*v_itr);

                using Item = envire::core::Item<ItemDataType>;
                using ItemItr = envire::core::EnvireGraph::ItemIterator<Item>;

                const std::pair<ItemItr, ItemItr> pair = control->graph->getItems<Item>(*v_itr);
#ifdef DEBUG
                if (pair.first == pair.second) {
                    LOG_DEBUG(("[EnvireSmurfLoader::loadNode] No " + type_name + " was found").c_str());
                }
#endif                 
                ItemItr i_itr;
                for(i_itr = pair.first; i_itr != pair.second; i_itr++)
                {
                    const ItemDataType &item_data = i_itr->getData();

#ifdef DEBUG
                    LOG_DEBUG(("[EnvireSmurfLoader::loadNode] " + type_name + " ***" + item_data.getName() + "*** was found" ).c_str());
#endif

                    // create NodeData
                    mars::interfaces::NodeData node = createNodeData(item_data);

                    // set Pose of NodeData according to Frame
                    setPose(frame_id, node);

                    // create SimNode with NodeData
                    createSimNode(node, item_data.getName(), frame_id); 
                }                
            }

            mars::interfaces::NodeData EnvireSmurfLoader::createNodeDataForFrame(const smurf::Frame &frame)
            {
                // create NodeData
                mars::interfaces::NodeData node;
                node.init(frame.getName());
                node.initPrimitive(mars::interfaces::NODE_TYPE_BOX, mars::utils::Vector(0.5, 0.5, 0.5), 0.00001);
                node.c_params.coll_bitmask = 0;
                node.movable = true;
                node.groupID = frame.getGroupId();
                node.density = 0.0;               
                
                return node; 
            }                 

            mars::interfaces::NodeData EnvireSmurfLoader::createNodeDataForCollidable(const smurf::Collidable &collidable)
            {
                urdf::Collision collision = collidable.getCollision();                
                // create NodeData
                NodeData node;
                node.init(collidable.getName());
                node.fromGeometry(collision.geometry);
                node.density = 0.0;
                node.mass = 0.00001;
                node.movable = true;
                node.c_params = collidable.getContactParams();
                node.groupID = collidable.getGroupId();         
                
                return node; 
            }            

            mars::interfaces::NodeData EnvireSmurfLoader::createNodeDataForInertial(const smurf::Inertial &inertial)
            {
                urdf::Inertial inertial_urd = inertial.getUrdfInertial();

                // create NodeData
                NodeData node;                    
                node.init(inertial.getName());
                node.initPrimitive(mars::interfaces::NODE_TYPE_SPHERE, mars::utils::Vector(0.1, 0.1, 0.1), inertial_urd.mass);
                node.groupID = inertial.getGroupId();
                node.movable = true;
                node.inertia[0][0] = inertial_urd.ixx;
                node.inertia[0][1] = inertial_urd.ixy;
                node.inertia[0][2] = inertial_urd.ixz;
                node.inertia[1][0] = inertial_urd.ixy;
                node.inertia[1][1] = inertial_urd.iyy;
                node.inertia[1][2] = inertial_urd.iyz;
                node.inertia[2][0] = inertial_urd.ixz;
                node.inertia[2][1] = inertial_urd.iyz;
                node.inertia[2][2] = inertial_urd.izz;
                node.inertia_set = true;
                node.c_params.coll_bitmask = 0;
                node.density = 0.0;         
                
                return node; 
            }                        

            void EnvireSmurfLoader::createSimNode(mars::interfaces::NodeData &node, std::string name, envire::core::FrameId frame_id)
            {
                // create NodePhysik
                mars::interfaces::NodeInterface* node_physics = mars::sim::PhysicsMapper::newNodePhysics(control->sim->getPhysics());

                bool instantiated = false;
                if(node.physicMode == NODE_TYPE_MLS) {
                    //instantiated = addMlsSurface(node.get());
                    LOG_ERROR("[EnvireSmurfLoader::createSimNode] NOT IMPLEMENTED NODE_TYPE_MLS");
                }
                else  {
                    instantiated = (node_physics->createNode(&node));
                }

                // create SimNode and add it into the graph
                if (instantiated) {                           
                    //NOTE Create and store also a simNode. The simNode interface is set to the physics node
                    mars::sim::SimNode *simNode = new mars::sim::SimNode(control, node); 
                    simNode->setInterface(node_physics);
                    std::shared_ptr<mars::sim::SimNode> simNodePtr(simNode);

                    using SimNodeItemPtr = envire::core::Item<std::shared_ptr<mars::sim::SimNode>>::Ptr;
                    using SimNodeItem =  envire::core::Item<std::shared_ptr<mars::sim::SimNode>>;

                    SimNodeItemPtr simNodeItem( new SimNodeItem(simNodePtr));        
                    control->graph->addItemToFrame(frame_id, simNodeItem);
#ifdef DEBUG
                    LOG_DEBUG(("[EnvireSmurfLoader::createSimNode] The SimNode is created for ***" + name + "***").c_str());
#endif                   
                } else {
                    LOG_ERROR(("[EnvireSmurfLoader::createSimNode] Failed to create SimNode for ***" + name + "***").c_str());
                }                     
            }

            void EnvireSmurfLoader::setPose(const envire::core::FrameId& frame, mars::interfaces::NodeData& node)
            {
                envire::core::Transform fromOrigin;
                if(center.compare(frame) == 0)
                {
                    //this special case happens when the graph only contains one frame
                    //and items are added to that frame. In that case asking the graph 
                    //for the transformation would cause an exception
                    fromOrigin.setTransform(base::TransformWithCovariance::Identity());
                }
                else
                {
                    fromOrigin = control->graph->getTransform(center, frame); 
                }
                node.pos = fromOrigin.transform.translation;
                node.rot = fromOrigin.transform.orientation;
            }  


            /*void EnvireSmurfLoader::loadJoints()
            {
#ifdef DEBUG
                LOG_DEBUG("[EnvireSmurfLoader::loadJoints] ------------------- Parse the graph and create SimJoints -------------------");
#endif                
                // search the graph
                envire::core::EnvireGraph::vertex_iterator v_itr, v_end;
                boost::tie(v_itr, v_end) = control->graph->getVertices();
                for(; v_itr != v_end; v_itr++)
                {              
#ifdef DEBUG          
                    envire::core::FrameId frame_id = control->graph->getFrameId(*v_itr);                
                    LOG_DEBUG(("[EnvireSmurfLoader::loadJoints] --- IN ***" + frame_id + "*** ---" ).c_str());
#endif 

                    loadJoint<smurf::Joint>(v_itr, "smurf::Joint");            
                    loadJoint<smurf::StaticTransformation>(v_itr, "smurf::StaticTransformation");
                }
            }          

            template <class ItemDataType>
            void EnvireSmurfLoader::loadJoint(envire::core::EnvireGraph::vertex_iterator v_itr, std::string type_name)
            {
                envire::core::FrameId frame_id = control->graph->getFrameId(*v_itr);

                using Item = envire::core::Item<ItemDataType>;
                using ItemItr = envire::core::EnvireGraph::ItemIterator<Item>;

                const std::pair<ItemItr, ItemItr> pair = control->graph->getItems<Item>(*v_itr);
#ifdef DEBUG
                if (pair.first == pair.second) {
                    LOG_DEBUG(("[EnvireSmurfLoader::loadJoint] No " + type_name + " was found").c_str());
                }
#endif                 
                ItemItr i_itr;
                for(i_itr = pair.first; i_itr != pair.second; i_itr++)
                {
                    const ItemDataType &item_data = i_itr->getData();

#ifdef DEBUG
                    LOG_DEBUG(("[EnvireSmurfLoader::loadJoint] " + type_name + " ***" + item_data.getName() + "*** was found" ).c_str());
#endif

                    // if the ItemDataType is smurf::Joint
                    // than create SimJoint first
                    if (std::is_same<ItemDataType, smurf::Joint>::value)
                    {
                        mars::sim::JointRecord* jointInfo(new mars::sim::JointRecord);
                        jointInfo->name = item_data.getName();
                        envire::core::Item<mars::sim::JointRecord>::Ptr jointItemPtr(new envire::core::Item<mars::sim::JointRecord>(*jointInfo));
                        control->graph->addItemToFrame(frame_id, jointItemPtr);                   

#ifdef DEBUG
                        LOG_DEBUG(("[EnvireSmurfLoader::createSimJoint] The SimJoint is created for ***" + item_data.getName() + "***").c_str());
#endif                        
                    }




                }                  
            }   

            void EnvireSmurfLoader::createJoint(smurf::Transformation& tranf)
            {
                std::shared_ptr<mars::sim::SimNode> sourceSim;
                std::shared_ptr<mars::sim::SimNode> targetSim;

                if (instantiable(smurfTf, sourceSim, targetSim))
                {
                    instantiate(smurfJoint, sourceSim, targetSim);
                } else {
                    LOG_ERROR(("[EnvireSmurfLoader::createSimJoint] The SimJoint can not be instantiate ***" + tranf.getName() + "***").c_str());
                }               
            }           

            bool EnvireJoints::instantiable(const smurf::Transformation& tranf, std::shared_ptr<mars::sim::SimNode>& sourceSim, std::shared_ptr<mars::sim::SimNode>& targetSim)
            {
                bool instantiable = true;
                std::string dependencyName = smurfJoint->getSourceFrame().getName();
                if (! getSimObject(dependencyName, sourceSim))
                {
                    instantiable = false;
                }
                dependencyName = smurfJoint->getTargetFrame().getName();
                if (! getSimObject(dependencyName, targetSim))
                {
                    instantiable = false;
                }
                return instantiable;
            }            */

        } // end of namespace EnvireSmurfLoader
    } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::EnvireSmurfLoader::EnvireSmurfLoader);
CREATE_LIB(mars::plugins::EnvireSmurfLoader::EnvireSmurfLoader);
