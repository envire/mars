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

#ifndef MARS_PLUGINS_SIMJOINTCREATOR_H
#define MARS_PLUGINS_SIMJOINTCREATOR_H

#ifdef _PRINT_HEADER_
  #warning "SimJointCreator.h"
#endif

#include <string>

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/MARSDefs.h>
#include <mars/interfaces/NodeData.h>

#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/JointManagerInterface.h>

#include <mars/sim/SimNode.h>
#include <mars/sim/SimJoint.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/GraphTypes.hpp>
#include <envire_core/items/Transform.hpp>
#include <envire_core/items/Item.hpp>

#include <smurf/Robot.hpp>

namespace mars {
  namespace plugins {
    namespace EnvireSmurfLoader {

        template <class ItemDataType>
        class SimJointCreator
        {
        protected:
            mars::interfaces::ControlCenter *control;
            envire::core::FrameId origin_frame_id;
            std::string type_name;

            virtual void createInternal(const ItemDataType &item_data, envire::core::FrameId frame_id) = 0;

            using Item = envire::core::Item<ItemDataType>;
            using ItemItr = envire::core::EnvireGraph::ItemIterator<Item>;              

        public:
            SimJointCreator(mars::interfaces::ControlCenter *control, envire::core::FrameId origin_frame_id, std::string type_name)
                : control(control), origin_frame_id(origin_frame_id), type_name(type_name)
            {}

            void create(envire::core::EnvireGraph::vertex_iterator v_itr) 
            {
                envire::core::FrameId frame_id = control->graph->getFrameId(*v_itr);

                const std::pair<ItemItr, ItemItr> pair = control->graph->getItems<Item>(*v_itr);
#ifdef DEBUG
                if (pair.first == pair.second) {
                    LOG_DEBUG(("[SimJointCreator::create] No " + type_name + " was found").c_str());
                }
#endif                 
                ItemItr i_itr;
                for(i_itr = pair.first; i_itr != pair.second; i_itr++)
                {
                    const ItemDataType &item_data = i_itr->getData();

#ifdef DEBUG
                    LOG_DEBUG(("[SimJointCreator::create] " + type_name + " ***" + item_data.getName() + "*** was found" ).c_str());
#endif

                    createInternal(item_data, frame_id);
                }    
            }

        protected:

            void setAnchor(const ItemDataType &item_data, mars::interfaces::JointData &jointData){
                envire::core::FrameId target_frame_id = item_data.getTargetFrame().getName();
                envire::core::Transform target_transf = control->graph->getTransform(origin_frame_id, target_frame_id); 
                utils::Vector anchor = target_transf.transform.translation;
                jointData.anchor = anchor;
                jointData.anchorPos = mars::interfaces::ANCHOR_NODE2;
            }  

            std::pair<mars::sim::SimNode*, mars::sim::SimNode*> findSourceTarget(const ItemDataType &item_data)
            {
                mars::sim::SimNode *source_sim_node = NULL, *target_sim_node = NULL;

                // check if source and target SimNode are already in the graph
                std::string source_frame_name = item_data.getSourceFrame().getName();
                if (!(source_sim_node = getSimNode(source_frame_name)))
                {
                    LOG_ERROR(("[SimJointCreator::findSimNodes] No Source SimNode in ***" + source_frame_name + "*** is found").c_str());
                }

                std::string target_frame_name = item_data.getTargetFrame().getName();
                if (!(target_sim_node = getSimNode(target_frame_name)))
                {
                    LOG_ERROR(("[SimJointCreator::findSimNodes] No Target SimNode in ***" + target_frame_name + "*** is found").c_str());
                }
#ifdef DEBUG
                LOG_DEBUG(("[SimJointCreator::findSimNodes] Source SimNode in ***" + source_frame_name + "*** and target SimNode in ***" + target_frame_name + "*** are found").c_str());
#endif                                 
                return std::pair<mars::sim::SimNode*, mars::sim::SimNode*>(source_sim_node, target_sim_node);
            }

            mars::sim::SimNode* getSimNode(const envire::core::FrameId &frame_name)
            {
                using Iterator = envire::core::EnvireGraph::ItemIterator<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>;
                Iterator begin, end;

                boost::tie(begin, end) = control->graph->getItems<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>(frame_name);
                if (begin != end){
                    return begin->getData().get();
                }
                return NULL;
            }                                 

            void createSimJoint(const ItemDataType &item_data, mars::interfaces::JointData &joint_data)
            {
                // create physical representation
                mars::sim::JointPhysics* jointPhysics(new mars::sim::JointPhysics(control->sim->getPhysics())); 
                std::shared_ptr<mars::interfaces::JointInterface> jointInterfacePtr(jointPhysics);

                std::pair<mars::sim::SimNode*, mars::sim::SimNode*> source_target = findSourceTarget(item_data);
                if (source_target.first != NULL && source_target.second != NULL)
                {

                    envire::core::FrameId storage_frame_id = item_data.getSourceFrame().getName();

                    // This is work around, to assign the source and target sim node to the joint data
                    // since the joint manager requires NodeID information
                    mars::interfaces::NodeId source_id = control->nodes->getID(source_target.first->getName());
                    if (source_id == INVALID_ID)
                        LOG_ERROR("[SimJointCreator::createSimJoint] Failed to find NodeIF ***" + source_target.first->getName() + "***");

                    mars::interfaces::NodeId target_id = control->nodes->getID(source_target.second->getName());
                    if (source_id == INVALID_ID)
                        LOG_ERROR("[SimJointCreator::createSimJoint] Failed to find NodeIF ***" + source_target.first->getName() + "***");                    

                    joint_data.nodeIndex1 = source_id;
                    joint_data.nodeIndex2 = target_id;
                    joint_data.frameID = storage_frame_id;

                    control->joints->addJoint(&joint_data);

                    /*if(jointInterfacePtr->createJoint(&joint_data, source_target.first->getInterface(), source_target.second->getInterface()))
                    {
    #ifdef DEBUG
                        LOG_DEBUG("[SimJointCreator::createSimJoint] Physical joint ***" + joint_data.name + "*** created");
    #endif
                        control->sim->sceneHasChanged(false);//important, otherwise the joint will be ignored by simulation

                        // create SimJoint               
                        std::shared_ptr<mars::sim::SimJoint> simJoint(new mars::sim::SimJoint(control, joint_data));
                        simJoint->setPhysicalJoint(jointInterfacePtr);

                        using SimJointPtrItemPtr = envire::core::Item<std::shared_ptr<mars::sim::SimJoint>>::Ptr;

                        envire::core::FrameId storage_frame_id = item_data.getSourceFrame().getName();

                        SimJointPtrItemPtr simJointPtrItemPtr(new envire::core::Item<std::shared_ptr<mars::sim::SimJoint>>(simJoint));
                        control->graph->addItemToFrame(storage_frame_id, simJointPtrItemPtr);            

    #ifdef DEBUG
                        LOG_DEBUG("[SimJointCreator::createSimJoint] SimJoint  ***" + joint_data.name + "*** is created");
    #endif                       
                        
                        addToJointRecord(storage_frame_id, joint_data.name, simJoint.get());                           
                    }
                    else
                    {
                        LOG_ERROR("[SimJointCreator::createSimJoint] Failed to create physical joint ***" + joint_data.name + "***");
                    } */                  
                }    
            }             

            void addToJointRecord(const envire::core::FrameId &frameId, const std::string &jointName, mars::sim::SimJoint *simJoint)
            {
                using RecordIterator = envire::core::EnvireGraph::ItemIterator<envire::core::Item<mars::sim::JointRecord>>;
                RecordIterator begin, end;
                // TODO Ask Arne, why to getItems I have to provide the template Item?
                // won't getItems only look for Items?
                boost::tie(begin, end) = control->graph->getItems<envire::core::Item<mars::sim::JointRecord>>(frameId);
                bool stored=false;
                while((!stored)&&(begin!=end))
                {
                    mars::sim::JointRecord* record = &(begin->getData());
                    if (record->name == jointName)
                    {
     #ifdef DEBUG
                        LOG_DEBUG("[SimJointCreator::addToJointRecord] Store SimJoint and JointInterface into JointRecord");
    #endif                                              
                        record->sim = std::shared_ptr<mars::sim::SimJoint>(simJoint);
                    }
                    begin ++;
                }

            }           
        };

        class SimJointCreatorJoint: public SimJointCreator<smurf::Joint>
        {
        public:
            SimJointCreatorJoint(mars::interfaces::ControlCenter *control, envire::core::FrameId origin_frame_id)
                : SimJointCreator(control, origin_frame_id, std::string("smurf::Joint"))
            {}            

        private:
            /**
            *   create JointRecord
            *   set axis1
            *   set anchor
            *   set Limits
            */
            virtual void createInternal(const smurf::Joint &joint, envire::core::FrameId frame_id)
            {
                // create JOINT
                mars::sim::JointRecord* joint_info(new mars::sim::JointRecord);
                joint_info->name = joint.getName();
                envire::core::Item<mars::sim::JointRecord>::Ptr jointItemPtr(new envire::core::Item<mars::sim::JointRecord>(*joint_info));
                control->graph->addItemToFrame(frame_id, jointItemPtr);                   

#ifdef DEBUG
                LOG_DEBUG(("[EnvireSmurfLoader::createInternal] The JointRecord is created for ***" + joint.getName() + "***").c_str());
#endif                        

                // create Joint
                mars::interfaces::JointData joint_data;
                joint_data.init(joint.getName(), getJointType(joint));

                // set Axis 1
                envire::core::FrameId target_frame_id = joint.getTargetFrame().getName();
                // get transformation from source to target
                envire::core::Transform target_transf = control->graph->getTransform(origin_frame_id, target_frame_id); 
                Eigen::Affine3d axis_transf = target_transf.transform.orientation * joint.getSourceToAxis().inverse();
                joint_data.axis1 = axis_transf.translation();

                // set Anchor
                setAnchor(joint, joint_data);

                // set Limits
                std::pair<double, double> position_limits = joint.getPositionLimits();
                joint_data.lowStopAxis1 = position_limits.first;
                joint_data.highStopAxis1 = position_limits.second;

                // DIRTY FIX!!! 
                // if the joints has spring (dynamic), than set the parameter in simulation
                if (joint.hasSpring())
                {
                    smurf::SpringParam spring_param = joint.getSpringParam();
                    joint_data.damping_const_constraint_axis1 = spring_param.damping_const_constraint_axis1;
                    joint_data.spring_const_constraint_axis1 = spring_param.spring_const_constraint_axis1;
                    joint_data.damping_constant = spring_param.springDamping;
                    joint_data.spring_constant = spring_param.springStiffness;
                }        

                // create Sim Joint
                createSimJoint(joint, joint_data);

                //jointData->print();
            }    

            mars::interfaces::JointType getJointType(const smurf::Joint &joint) {
                boost::shared_ptr<urdf::Joint> joint_model = joint.getJointModel();

                std::string log_type;
                mars::interfaces::JointType joint_type;

                switch (joint_model->type)
                {
                  case urdf::Joint::FIXED:
                  {
                    log_type = "Fixed";
                    joint_type = mars::interfaces::JOINT_TYPE_FIXED; // use consts so that all are defined only once
                    break;
                  }
                  case urdf::Joint::FLOATING:
                  {
                    //TODO We have some but seem not to be supported by mars?  Ask
                    log_type = "Floating";
                    joint_type = mars::interfaces::JOINT_TYPE_FIXED;
                    break;
                  }
                  case urdf::Joint::CONTINUOUS:
                  {
                    // We have some
                    log_type = "Continuous"; 
                    joint_type = mars::interfaces::JOINT_TYPE_HINGE;
                    break;
                  }
                  case urdf::Joint::PRISMATIC:
                  {
                    log_type = "Prismatic"; 
                    joint_type = mars::interfaces::JOINT_TYPE_SLIDER;
                    break;
                  }
                  case urdf::Joint::REVOLUTE:
                  {
                    // We have some
                    log_type = "Revolute"; 
                    joint_type = mars::interfaces::JOINT_TYPE_HINGE;
                    break;
                  }
                  case urdf::Joint::PLANAR:
                  {
                    log_type = "Planar"; 
                    // TODO No support? Set fixed
                    joint_type = mars::interfaces::JOINT_TYPE_FIXED;
                    break;
                  }
                  case urdf::Joint::UNKNOWN:
                  default:
                  {
                    log_type = "Unknown"; 
                    joint_type = mars::interfaces::JOINT_TYPE_FIXED;
                    break;
                  }
                }
#ifdef DEBUG
                LOG_DEBUG("[SimJointCreatorJoint::getJointType] The joint type is: " + log_type); 
#endif
                return joint_type;
            }                      
        };

        class SimJointCreatorStaticTranf: public SimJointCreator<smurf::StaticTransformation>
        {
        public:
            SimJointCreatorStaticTranf(mars::interfaces::ControlCenter *control, envire::core::FrameId origin_frame_id)
                : SimJointCreator(control, origin_frame_id, std::string("smurf::StaticTransformation"))
            {}            

        private:
            /**
            *   set anchor
            */            
            virtual void createInternal(const smurf::StaticTransformation &static_transf, envire::core::FrameId frame_id)
            {
                                // create Joint
                mars::interfaces::JointData joint_data;
                //joint_data.init(static_transf.getName(), getJointType(static_transf));  ????
                joint_data.init(static_transf.getName(), mars::interfaces::JOINT_TYPE_FIXED);

#ifdef DEBUG
                LOG_DEBUG("[SimJointCreatorJoint::createInternal] The joint type is: Fixed"); 
#endif                

                // set Anchor
                setAnchor(static_transf, joint_data);              

                createSimJoint(static_transf, joint_data);
            }           
        };        


    } // end of namespace EnvireSmurfLoader
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_ENVIRESMURFLOADER
