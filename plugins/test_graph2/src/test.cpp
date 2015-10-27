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
    namespace test_graph2 {

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
        
        /*Construct the following graph
         *       a
         *       |
         *       b
         *     /   \
         *    c     d
         * 
         */
        
        
        FrameId a = "a";
        FrameId b = "b";
        FrameId c = "c";
        FrameId d = "d";
        
        Transform tf;
        tf.transform.translation << 0, 0, 2;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(a, b, tf);
        
        tf.transform.translation << 3, 0, 2;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(b, c, tf);
        
        tf.transform.translation << -3, 0, 2;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(b, d, tf);
        
        
        
        NodeData data;
        data.init("node", Vector(0,0,0));
        data.initPrimitive(NODE_TYPE_BOX, Vector(0.4, 0.4, 0.4), 0.1);
        data.movable = false;
        ConfigMapItem::Ptr item(new ConfigMapItem);
        data.toConfigMap(&(item.get()->getData()));
        control->graph->addItemToFrame(a, item);
        
        NodeData data2;
        data2.init("node2", Vector(0,0,0));
        data2.initPrimitive(NODE_TYPE_BOX, Vector(0.4, 0.4, 0.4), 0.1);
        data2.movable = false;
        ConfigMapItem::Ptr item2(new ConfigMapItem);
        data2.toConfigMap(&(item2.get()->getData()));
        control->graph->addItemToFrame(b, item2);
        
        NodeData data3;
        data3.init("node3", Vector(0,0,0));
        data3.initPrimitive(NODE_TYPE_BOX, Vector(0.4, 0.4, 0.4), 0.1);
        data3.movable = false;
        ConfigMapItem::Ptr item3(new ConfigMapItem);
        data3.toConfigMap(&(item3.get()->getData()));
        control->graph->addItemToFrame(c, item3);
        
        NodeData data4;
        data4.init("node4", Vector(0,0,0));
        data4.initPrimitive(NODE_TYPE_BOX, Vector(0.4, 0.4, 0.4), 0.1);
        data4.movable = false;
        ConfigMapItem::Ptr item4(new ConfigMapItem);
        data4.toConfigMap(&(item4.get()->getData()));
        control->graph->addItemToFrame(d, item4);     
      }

      void TestGraph2::reset() {
      }

      TestGraph2::~TestGraph2() {
      
      }


      void TestGraph2::update(sReal time_ms) 
      {
        Transform tf = control->graph->getTransform("a", "b");
        tf.transform.orientation *= base::Quaterniond(base::AngleAxisd(0.00054, base::Vector3d(0, 0, 1)));
        control->graph->updateTransform("a", "b", tf);
      }

      void TestGraph2::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
      }

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test_graph2::TestGraph2);
CREATE_LIB(mars::plugins::test_graph2::TestGraph2);
