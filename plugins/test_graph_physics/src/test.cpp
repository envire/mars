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
        FrameId ball = "ball";

        
        Transform tf;
        tf.transform.translation << 0, 0, 10;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(floor, ball, tf);      
        
        NodeData data;
        data.init("floorData", Vector(0,0,0));
        data.initPrimitive(interfaces::NODE_TYPE_BOX, Vector(5, 5, 0.3), 0.1);
        data.movable = false;
        boost::shared_ptr<ConfigMapItem> item(new ConfigMapItem);
        data.toConfigMap(&(item.get()->getData()));
        control->graph->addItemToFrame(floor, item);
        NodeData data2;
        data2.init("ballData", Vector(0,0,0));
        data2.initPrimitive(interfaces::NODE_TYPE_SPHERE, Vector(0.3, 0.3, 0.3), 0.1);
        data2.movable = true;
        boost::shared_ptr<ConfigMapItem> item2(new ConfigMapItem);
        data2.toConfigMap(&(item2.get()->getData()));
        control->graph->addItemToFrame(ball, item2); 

        //jointedItems();
        
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
        FrameId id = boost::lexical_cast<FrameId>(itemId);
        ++itemId;
        std::cout << "item count: " << itemId << std::endl;
        boost::random::uniform_int_distribution<> rnd(-50, 50);
        const float x = rnd(rng)/ 10.0;
        const float y = rnd(rng)/ 10.0;
        Transform tf;
        tf.transform.translation << x, y, 10;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(floor, id, tf);    
        NodeData data = randomNodeData(id);
        boost::shared_ptr<ConfigMapItem> item(new ConfigMapItem);
        data.toConfigMap(&(item.get()->getData()));
        control->graph->addItemToFrame(id, item); 
        
      }

      FrameId TestGraph2::getNextFrameId()
      {
	++itemId;
        FrameId id = boost::lexical_cast<FrameId>(itemId);
        return id;
      }

      void TestGraph2::addNodeToFrame(FrameId id, NodeData data)
      {
        // Create a pointer to an item that stores a configmap
        boost::shared_ptr<ConfigMapItem> item(new ConfigMapItem);
        // Totaly crazy way to set the data of the Item
        data.toConfigMap(&(item.get()->getData()));
	//configmaps::ConfigMap myConfigMap;
	//data.toConfigMap(&myConfigMap);
	//std::cout << "Before puting in the graph" << item.get()->getData()["name"] << std::endl;
        control->graph->addItemToFrame(id, item); 
      }

      void TestGraph2::addJointToFrame(FrameId id, JointData data)
      {
        boost::shared_ptr<ConfigMapItem> item(new ConfigMapItem);
        data.toConfigMap(&(item.get()->getData()));
        control->graph->addItemToFrame(id, item); 
      }

      void TestGraph2::jointedItems()
      {
        // Create two linked objects
        // Create frame and transformation for the Envire Graph
        FrameId id = getNextFrameId();
        int object1Id = itemId;
        Transform tf;
        tf.transform.translation << 1, 1, 10;
        tf.transform.orientation = base::Quaterniond::Identity();
        // Add transformation to frame from floor (root)
        control->graph->addTransform(floor, id, tf); 
        // Put the information about the object in an item and include it in the graph
        NodeData data = randomNodeData(id);
        addNodeToFrame(id, data);
        // Same for the second object
        id = getNextFrameId();
        int object2Id = itemId;
        tf.transform.translation << 2, 2, 10;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(floor, id, tf);
        data = randomNodeData(id);
        addNodeToFrame(id, data);
        // Add the joint
        JointData jointData;
        std::string name = "joint";
        // The joint should put together the two last create objects
        std::cout << "object1Id: " << object1Id << std::endl;
        std::cout << "object2Id: " << object2Id << std::endl;
        jointData.init(name, interfaces::JOINT_TYPE_FIXED, object1Id, object2Id);
        id = getNextFrameId();
        tf.transform.translation << 1.5, 1.5, 10;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(floor, id, tf);
        addJointToFrame(id, jointData);
      }
      

      void TestGraph2::reset() {
      }

      TestGraph2::~TestGraph2() {
      
      }


      void TestGraph2::update(sReal time_ms) 
      {
          dropItem();
      }

      void TestGraph2::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
      }

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test_graph_physics::TestGraph2);
CREATE_LIB(mars::plugins::test_graph_physics::TestGraph2);
