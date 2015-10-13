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
    namespace test_graph_physics2 {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace envire::core;
      using namespace mars::sim;
      using namespace std;

      TestGraphPhysics2::TestGraphPhysics2(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "TestGraphPhysics2"){
      }
  
      void TestGraphPhysics2::init() 
      {
        FrameId floor = "floor";
        FrameId ball = "ball";
        FrameId fixedBall = "fixedBall";
        FrameId ball2 = "ball2";
        FrameId fixedBall2 = "fixedBall2";
        //the balls should fall to the floor while the fixed balls should
        //stay at the same position.
        //tree:

         /*      floor
         *         |
         *        ball
         *         |
         *      fixedBall
         *         |
         *        ball2
         *         |
         *      fixedBall2    */
        
        
        
        Transform floorToBall;
        floorToBall.transform.translation << 0, 0, 10;
        floorToBall.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(floor, ball, floorToBall);    
        
        Transform ballToFixedBall;
        ballToFixedBall.transform.translation << 0, 3, 0;
        ballToFixedBall.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(ball, fixedBall, ballToFixedBall);   
        
        Transform fixedBallToBall2;
        fixedBallToBall2.transform.translation << 4, 0, 1;
        fixedBallToBall2.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(fixedBall, ball2, fixedBallToBall2);   
        
        Transform ball2ToFixedBall2;
        ball2ToFixedBall2.transform.translation << 4, 3, 2;
        ball2ToFixedBall2.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(ball2, fixedBall2, ball2ToFixedBall2);   
        
        
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
        
        NodeData data3;
        data3.init("ballData2", Vector(0,0,0));
        data3.initPrimitive(NODE_TYPE_SPHERE, Vector(0.3, 0.3, 0.3), 0.1);
        data3.movable = true;
        boost::shared_ptr<ConfigMapItem> item3(new ConfigMapItem);
        data3.toConfigMap(&(item3.get()->getData()));
        control->graph->addItemToFrame(ball2, item3);
        
        NodeData data4;
        data4.init("ballData3", Vector(0,0,0));
        data4.initPrimitive(NODE_TYPE_SPHERE, Vector(0.3, 0.3, 0.3), 0.1);
        data4.movable = false;
        boost::shared_ptr<ConfigMapItem> item4(new ConfigMapItem);
        data4.toConfigMap(&(item4.get()->getData()));
        control->graph->addItemToFrame(fixedBall, item4); 
        
        NodeData data5;
        data5.init("ballData4", Vector(0,0,0));
        data5.initPrimitive(NODE_TYPE_SPHERE, Vector(0.3, 0.3, 0.3), 0.1);
        data5.movable = false;
        boost::shared_ptr<ConfigMapItem> item5(new ConfigMapItem);
        data5.toConfigMap(&(item5.get()->getData()));
        control->graph->addItemToFrame(fixedBall2, item5); 
        
      }
      
      

      void TestGraphPhysics2::reset() {
      }

      TestGraphPhysics2::~TestGraphPhysics2() {
      
      }


      void TestGraphPhysics2::update(sReal time_ms) 
      {
 
      }

      void TestGraphPhysics2::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
      }

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test_graph_physics2::TestGraphPhysics2);
CREATE_LIB(mars::plugins::test_graph_physics2::TestGraphPhysics2);
