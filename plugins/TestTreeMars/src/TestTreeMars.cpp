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
 * \file TestTreeMars.cpp
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Plugin
 *
 * Version 0.1
 */


#include "TestTreeMars.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <envire_core/Transform.hpp>
#include <base/TransformWithCovariance.hpp>
#include <stdlib.h>
#include <sstream>

namespace mars {
  namespace plugins {
    namespace TestTreeMars {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace mars::sim;

      TestTreeMars::TestTreeMars(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "TestTreeMars"), drawingId(0) {
      }
  
      void TestTreeMars::init() {
        NodeData boxNode;
        boxNode.init("box", //name
                      Vector(0,0,-1)); //position

        //we want a primitive object, not a mesh or something similar
        boxNode.initPrimitive(NODE_TYPE_BOX,//node type (box, sphere, etc.)
                               Vector(10, 10, 0.1),//extents (width, height, length)
                               0.1);//mass
        boxNode.movable = false;
        boxNode.material.transparency = 0.5;
        base::TransformWithCovariance tf;
        tf.translation << 0, 0, -1;
        Transform t;
        t.setTransform(tf);
        control->tree->addObject("testObject", boxNode, t);

        for(int i = 0; i < 500; ++i)
        {
          std::stringstream ss;
          ss << "Ball " << i;
          addBall(ss.str());
        }

        control->tree->drawDotFile("0000.dot");

      }

      void TestTreeMars::addBall(const std::string& name) {


        const double x = ((rand() % 40) - 20)/10.0;
        const double y = ((rand() % 40) - 20)/10.0;
        const double z = (rand() % 40)/10.0;
        NodeData sphereNode;
        sphereNode.init("sphere", //name
                      Vector(x,y,z)); //position

        //we want a primitive object, not a mesh or something similar
        sphereNode.initPrimitive(NODE_TYPE_SPHERE,//node type (box, sphere, etc.)
                               Vector(0.1, 0.1, 0.1),//extents (width, height, length)
                               0.1);//mass
        sphereNode.movable = true;
        base::TransformWithCovariance tf;
        tf.translation << x, y, z;
        envire::core::Transform t;
        t.setTransform(tf);
        control->tree->addObject(name, sphereNode, t);

      }

      void TestTreeMars::reset() {
      }

      TestTreeMars::~TestTreeMars() {
      }


      void TestTreeMars::update(sReal time_ms) {
        //uncomment this to get one dot file for each step
     //   drawingId++;
     //   std::stringstream ss;
     //   ss << std::setfill('0') << std::setw(4) << drawingId << ".dot";
     //   control->tree->drawDotFile(ss.str());
      }

      void TestTreeMars::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
      }
  
      void TestTreeMars::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::TestTreeMars::TestTreeMars);
CREATE_LIB(mars::plugins::TestTreeMars::TestTreeMars);
