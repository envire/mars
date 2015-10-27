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
        //construct a small solar system
        
        
        
        FrameId sun = "sun";
        FrameId earth = "earth";
        FrameId moon = "moon";
        FrameId venus = "venus";
        
        Transform tf;
        tf.transform.translation << 4, 0, 0;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(sun, earth, tf);
        
        tf.transform.translation << 1, 0, 0;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(earth, moon, tf);
        
        tf.transform.translation << 2, 0, 0;
        tf.transform.orientation = base::Quaterniond::Identity();
        control->graph->addTransform(sun, venus, tf);
        
        
        
        NodeData data;
        data.init("sun", Vector(0,0,0));
        data.initPrimitive(NODE_TYPE_SPHERE, Vector(0.5, 0.5, 0.5), 0.1);
        data.movable = false;
        ConfigMapItem::Ptr item(new ConfigMapItem);
        data.toConfigMap(&(item.get()->getData()));
        control->graph->addItemToFrame(sun, item);
        
        NodeData data2;
        data2.init("earth", Vector(0,0,0));
        data2.initPrimitive(NODE_TYPE_SPHERE, Vector(0.3, 0.3, 0.3), 0.1);
        data2.movable = false;
        ConfigMapItem::Ptr item2(new ConfigMapItem);
        data2.toConfigMap(&(item2.get()->getData()));
        control->graph->addItemToFrame(earth, item2);
        
        NodeData data3;
        data3.init("moon", Vector(0,0,0));
        data3.initPrimitive(NODE_TYPE_SPHERE, Vector(0.1, 0.1, 0.1), 0.1);
        data3.movable = false;
        ConfigMapItem::Ptr item3(new ConfigMapItem);
        data3.toConfigMap(&(item3.get()->getData()));
        control->graph->addItemToFrame(moon, item3);
        
        NodeData data4;
        data4.init("venus", Vector(0,0,0));
        data4.initPrimitive(NODE_TYPE_SPHERE, Vector(0.4, 0.4, 0.4), 0.1);
        data4.movable = false;
        ConfigMapItem::Ptr item4(new ConfigMapItem);
        data4.toConfigMap(&(item4.get()->getData()));
        control->graph->addItemToFrame(venus, item4);     
      }

      void TestGraph2::reset() {
      }

      TestGraph2::~TestGraph2() {
      
      }


      void TestGraph2::update(sReal time_ms) 
      {
        const sReal time_s = time_ms / 1000.0;
        const sReal earthRotPerSecond = 0.0034;
        const sReal venusRotPerSecond = 0.0012;
        const sReal moonRotPerSecond = 0.006;
        const sReal earthRot = earthRotPerSecond * time_s;
        const sReal venusRot = venusRotPerSecond * time_s;
        const sReal moonRot = moonRotPerSecond * time_s;
        
        Transform sunToEarth = control->graph->getTransform("sun", "earth");
        sunToEarth.transform.translation = base::AngleAxisd(earthRot, base::Vector3d(0, 0, 1)) * sunToEarth.transform.translation;
        control->graph->updateTransform("sun", "earth", sunToEarth);
        
        Transform sunToVenus = control->graph->getTransform("sun", "venus");
        sunToVenus.transform.translation = base::AngleAxisd(venusRot, base::Vector3d(0, 0, 1)) * sunToVenus.transform.translation;
        control->graph->updateTransform("sun", "venus", sunToVenus);

        Transform earthToMoon = control->graph->getTransform("earth", "moon");
        earthToMoon.transform.translation = base::AngleAxisd(moonRot, base::Vector3d(0, 0, 1)) * earthToMoon.transform.translation;
        control->graph->updateTransform("earth", "moon", earthToMoon);
      }

      void TestGraph2::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
      }

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test_graph2::TestGraph2);
CREATE_LIB(mars::plugins::test_graph2::TestGraph2);
