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

#include <mars/sim/PhysicsMapper.h>
#include <mars/sim/SimNode.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphTypes.hpp>
#include <envire_core/items/Transform.hpp>
#include <envire_core/items/Item.hpp>

#include <smurf/Robot.hpp>

#define DEBUG

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

                    // set Pose of NodeData according to Frame
                    setPose(node_data, frame_id);

                    // create SimNode with NodeData
                    createSimNode(node_data, frame_id, item_data.getName()); 
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

                    std::cout << "TEEEEEEEEST" << std::endl;
                    simNode->getInterface();

                    using SimNodeItemPtr = envire::core::Item<std::shared_ptr<mars::sim::SimNode>>::Ptr;
                    using SimNodeItem =  envire::core::Item<std::shared_ptr<mars::sim::SimNode>>;

                    SimNodeItemPtr simNodeItem( new SimNodeItem(simNodePtr));        
                    control->graph->addItemToFrame(frame_id, simNodeItem);

                    std::cout << "TEEEEEEEEST2" << std::endl;
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
                // create NodeData
                mars::interfaces::NodeData node;
                node.init(collidable.getName());
                node.fromGeometry(collision.geometry);
                node.density = 0.0;
                node.mass = 0.00001;
                node.movable = true;
                node.c_params = collidable.getContactParams();
                node.groupID = collidable.getGroupId();         
                
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

                // create NodeData
                mars::interfaces::NodeData node;                    
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
        };

    } // end of namespace EnvireSmurfLoader
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_ENVIRESMURFLOADER
