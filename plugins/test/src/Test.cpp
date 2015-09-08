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
 * \file Test.cpp
 * \author Yong-Ho Yoo
 * \brief 
 *
 * Version 0.1
 */


#include "Test.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include "../../../sim/src/core/TreeMars.h"

#include <boost/test/unit_test.hpp>
#include <mars/interfaces/sim/ControlCenter.h>
#include <envire_core/GraphViz.hpp>

namespace mars {
  namespace plugins {
    namespace test {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace mars::sim;
      
using namespace envire::core;
using mars::sim::TreeMars;
using mars::interfaces::ControlCenter;
using base::TransformWithCovariance;
using envire::core::GraphViz;
using mars::interfaces::NodeData;

      Test::Test(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "Test") {
      }
  
      void Test::init() {
  
          TreeMars* treeMars = new TreeMars(control);
          TreeMarsInterface* treeMarsInterface = treeMars;
          control->tree = treeMarsInterface;
          
      NodeData node;    
      NodeData *nodeS = &node;

      printf("Pointer assignation\n");

      const std::string &name = "box";
      nodeS->pos = utils::Vector::Zero();
      nodeS->name = name;
      nodeS->movable = true;
      nodeS->physicMode = NODE_TYPE_BOX;
      nodeS->index=1;
      nodeS->noPhysical = false;
      nodeS->inertia_set=false;
      nodeS->ext.x() = 1;
      nodeS->ext.y() = 1; 
      nodeS->ext.z() = 1;   
      NodeType type = nodeS->physicMode;
      nodeS->mass = 1.0f;
      nodeS->initPrimitive(type, nodeS->ext, nodeS->mass);
     
     
     
    for(int i = 0; i < 2; ++i)
    {
      TransformWithCovariance tf;
      tf.translation << i, i * 2, i * 3;
      Transform t;
      t.setTransform(tf);
      control->tree->addObject("test " + boost::lexical_cast<std::string>(i), nodeS, t);
    }
          
     
    
      }

      void Test::reset() {
      }

      Test::~Test() {
      }


      void Test::update(sReal time_ms) {

        // control->motors->setMotorValue(id, value);

        
        Vector pos;
        pos = control->items->getPosition(1);
        //pos = control->nodes->getPosition(1);        
        printf("pos.z = ..........%f\n", pos.z());
        
      }

      void Test::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        printf("plugin ... receiveData\n");										
        // package.get("force1/x", force);
      }
  
      void Test::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
		  
        printf("plugin ... cfgUpdateProperty\n");

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

    } // end of namespace test
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test::Test);
CREATE_LIB(mars::plugins::test::Test);
