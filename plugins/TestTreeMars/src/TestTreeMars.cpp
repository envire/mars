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

namespace mars {
  namespace plugins {
    namespace TestTreeMars {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace mars::sim;

      TestTreeMars::TestTreeMars(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "TestTreeMars") {
      }
  
      void TestTreeMars::init() {
        NodeData nodeData;
        nodeData.init("box", //name
                      Vector(1,2,3)); //position

        //we want a primitive object, not a mesh or something similar
        nodeData.initPrimitive(NODE_TYPE_BOX,//node type (box, sphere, etc.)
                               Vector(3, 1, 1),//extents (width, height, length)
                               1);//mass
        nodeData.movable = true;
        base::TransformWithCovariance tf;
        tf.translation << 5, 1, 1;
        envire::core::Transform t;
        t.setTransform(tf);


        control->tree->addObject("testObject", nodeData, t);
      }

      void TestTreeMars::reset() {
      }

      TestTreeMars::~TestTreeMars() {
      }


      void TestTreeMars::update(sReal time_ms) {

        // control->motors->setMotorValue(id, value);
        //call update here
      }

      void TestTreeMars::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
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
