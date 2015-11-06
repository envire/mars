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
 * \file test.cpp
 * \author Arne Böckmann
 * \brief Plugin
 */


#include "test.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <envire_core/items/Transform.hpp>
#include <envire_core/graph/TransformGraph.hpp>
#include <mars/sim/ConfigMapItem.h>
#include <base/TransformWithCovariance.hpp>
#include <stdlib.h>
#include <string>
#include <boost/lexical_cast.hpp>


namespace mars {
  namespace plugins {
    namespace test_graph_physics {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace envire::core;
      using namespace mars::sim;
      using namespace std;

      TestGraph2::TestGraph2(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "TestGraph2"){
      }
  
      void TestGraph2::init() 
      {
        itemId = 0;
        
        
        floor = "floor";
        control->graph->addFrame(floor);
        
        NodeData data;
        data.init("floorData", Vector(0,0,0));
        data.initPrimitive(interfaces::NODE_TYPE_BOX, Vector(5, 5, 0.3), 0.1);
        data.movable = false;
        PhysicsConfigMapItem::Ptr item(new PhysicsConfigMapItem);
        data.toConfigMap(&(item.get()->getData()));
        control->graph->addItemToFrame(floor, item);


        jointedItems();
      }
      
      NodeData TestGraph2::randomNodeData(FrameId id)
      {
        NodeData data;
        data.init(id + "data", Vector(0,0,0));
        mars::interfaces::NodeType type;
        switch(boost::random::uniform_int_distribution<>(0, 3)(rng))
        {
          case 0:
            type = interfaces::NODE_TYPE_BOX;
            break;
          case 1:
            type = interfaces::NODE_TYPE_CAPSULE;
            break;
          case 2:
            type = interfaces::NODE_TYPE_CYLINDER;
            break;
          case 3:
            type = interfaces::NODE_TYPE_SPHERE;
            break;
          default:
            break;
        }
        data.initPrimitive(type, Vector(0.3, 0.3, 0.3), 0.1);
        data.movable = true;
        return data;
      }

      void TestGraph2::dropItem()
      {
        FrameId id = getNextFrameId();
        boost::random::uniform_int_distribution<> rnd(-50, 50);
        const float x = rnd(rng)/ 10.0;
        const float y = rnd(rng)/ 10.0;
        Transform tf;
        tf.transform.translation << x, y, 10;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(floor, id, tf);    
        NodeData data = randomNodeData(id);
        PhysicsConfigMapItem::Ptr item(new PhysicsConfigMapItem);
        data.toConfigMap(&(item.get()->getData()));
        control->graph->addItemToFrame(id, item); 
        
      }

      FrameId TestGraph2::getNextFrameId()
      {
        ++itemId;
        FrameId id = boost::lexical_cast<FrameId>(itemId);
        return id;
      }

      boost::uuids::uuid TestGraph2::addNodeToFrame(FrameId id, NodeData data)
      {
        // Create a pointer to an item that stores a configmap
        PhysicsConfigMapItem::Ptr item(new PhysicsConfigMapItem);
        // Totaly crazy way to set the data of the Item
        data.toConfigMap(&(item.get()->getData()));
        //configmaps::ConfigMap myConfigMap;
        //data.toConfigMap(&myConfigMap);
        //std::cout << "Before puting in the graph" << item.get()->getData()["name"] << std::endl;
        control->graph->addItemToFrame(id, item); 
        return item->getID();
      }

      void TestGraph2::addJointToFrame(FrameId id, JointData& data,
          boost::uuids::uuid itemId1, boost::uuids::uuid itemId2)
      {
        JointConfigMapItem::Ptr item(new JointConfigMapItem);
        //parse the joint data into a config map
        data.toConfigMap(&(item.get()->getData()));
        //the physics plugin needs to know which items should be connected
        //therefore we add that data to the config map as well
        //The ConfigMap can still be parsed into a JointData, these values will
        //just be ignored
        item->getData()["itemId1"][0] = configmaps::ConfigItem(boost::lexical_cast<string>(itemId1));
        item->getData()["itemId2"][0] = configmaps::ConfigItem(boost::lexical_cast<string>(itemId2));
        control->graph->addItemToFrame(id, item); 
      }

      void TestGraph2::jointedItems()
      {
        //floor -> id1 -> joint -> id2
        FrameId id1 = getNextFrameId();
        FrameId id2 = getNextFrameId();
        FrameId jointId = "joint_" + getNextFrameId();
        
        //floor -> id1
        Transform floorToId1;
        floorToId1.transform.translation << 1, 1, 10;
        floorToId1.transform.orientation = base::Quaterniond::Identity();        
        control->graph->addTransform(floor, id1, floorToId1);         
        NodeData data = randomNodeData(id1);
        boost::uuids::uuid uuid1 = addNodeToFrame(id1, data);

        //id1 -> joint (the joint is fixed and located half way between id1 and id2)
        Transform id1ToJoint;
        id1ToJoint.transform.translation << 2, 2, 0;
        id1ToJoint.transform.orientation = base::Quaterniond::Identity(); 
        control->graph->addTransform(id1, jointId, id1ToJoint);
        
        //joint -> id2 (has to be done before we can add the JointData to joint
        //              because we need the uuid of id2 item)
        Transform jointToId2;
        jointToId2.transform.translation << 2, 2, 0;
        jointToId2.transform.orientation = base::Quaterniond::Identity();        
        control->graph->addTransform(jointId, id2, jointToId2);      
        data = randomNodeData(id2);
        boost::uuids::uuid uuid2 = addNodeToFrame(id2, data);
        
        // Add the joint
        JointData jointData;
        jointData.init("joint", interfaces::JOINT_TYPE_FIXED, 0, 0);
        addJointToFrame(jointId, jointData, uuid1, uuid2);
      }
      
      void TestGraph2::reset() {
      }

      TestGraph2::~TestGraph2() {
      
      }

      void TestGraph2::update(sReal time_ms) 
      {
       //   dropItem();
      }

      void TestGraph2::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
      }

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test_graph_physics::TestGraph2);
CREATE_LIB(mars::plugins::test_graph_physics::TestGraph2);
