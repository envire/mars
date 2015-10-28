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

      void TestGraph2::addNodeToFrame(FrameId id, NodeData data)
      {
        // Create a pointer to an item that stores a configmap
        PhysicsConfigMapItem::Ptr item(new PhysicsConfigMapItem);
        // Totaly crazy way to set the data of the Item
        data.toConfigMap(&(item.get()->getData()));
        //configmaps::ConfigMap myConfigMap;
        //data.toConfigMap(&myConfigMap);
        //std::cout << "Before puting in the graph" << item.get()->getData()["name"] << std::endl;
        control->graph->addItemToFrame(id, item); 
      }

      void TestGraph2::addJointToFrame(FrameId jointId, FrameId id, const FrameId& id2, JointData data)
      {
        JointConfigMapItem::Ptr item(new JointConfigMapItem);
        //parse the joint data into a config map
        data.toConfigMap(&(item.get()->getData()));
        //the physics plugin needs to know which frames should be connected
        //therefore we add that data to the config map as well
        //The ConfigMap can still be parsed into a JointData, these values will
        //just be ignored
        item->getData()["frameId1"][0] = configmaps::ConfigItem(id);
        item->getData()["frameId2"][0] = configmaps::ConfigItem(id2);
        control->graph->addItemToFrame(jointId, item); 
      }

      void TestGraph2::jointedItems()
      {
        // Create two linked objects
        // Create frame and transformation for the Envire Graph
        FrameId id1 = getNextFrameId();
        Transform tf;
        tf.transform.translation << 1, 1, 10;
        tf.transform.orientation = base::Quaterniond::Identity();
        // Add transformation to frame from floor (root)
        control->graph->addTransform(floor, id1, tf); 
        // Put the information about the object in an item and include it in the graph
        NodeData data = randomNodeData(id1);
        addNodeToFrame(id1, data);
        
        // Same for the second object
        FrameId id2 = getNextFrameId();
        tf.transform.translation << 6, 6, 10;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(floor, id2, tf);
        data = randomNodeData(id2);
        addNodeToFrame(id2, data);
        
        // Add the joint
        JointData jointData;
        std::string name = "joint";
        // The joint should put together the two last create objects
        //the object ids are not initialized here because we do not know the 
        //phyics ids of the items (and we do not want to know them)
        //FIXME we are putting information into the configMap that we dont need
        // (the object ids)
        jointData.init(name, interfaces::JOINT_TYPE_FIXED, 0, 0);
        
        FrameId jointId = getNextFrameId();
        //FIXME why do we store the joint in its own frame?!
        tf.transform.translation << 1.5, 1.5, 10;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(floor, jointId, tf);
        addJointToFrame(jointId, id1, id2, jointData);
      }
      

      void TestGraph2::reset() {
      }

      TestGraph2::~TestGraph2() {
      
      }


      void TestGraph2::update(sReal time_ms) 
      {
          //dropItem();
      }

      void TestGraph2::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
      }

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test_graph_physics::TestGraph2);
CREATE_LIB(mars::plugins::test_graph_physics::TestGraph2);
