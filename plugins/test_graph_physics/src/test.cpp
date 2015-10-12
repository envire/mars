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
        //construct a small solar system
        
        
        
        FrameId floor = "floor";
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
        data2.initPrimitive(NODE_TYPE_SPHERE, Vector(0.3, 0.3, 0.3), 0.1);
        data2.movable = true;
        boost::shared_ptr<ConfigMapItem> item2(new ConfigMapItem);
        data2.toConfigMap(&(item2.get()->getData()));
        control->graph->addItemToFrame(ball, item2); 
      }

      void TestGraph2::reset() {
      }

      TestGraph2::~TestGraph2() {
      
      }


      void TestGraph2::update(sReal time_ms) 
      {

      }

      void TestGraph2::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
      }

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test_graph_physics::TestGraph2);
CREATE_LIB(mars::plugins::test_graph_physics::TestGraph2);
