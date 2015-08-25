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
 * \author tes (tes)
 * \brief test
 *
 * Version 0.1
 */


#include "Test.h"
#include <mars/interfaces/sim/ItemManagerInterface.h>
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <envire_core/Item.hpp>
//#include "EnvireItem.hpp"

namespace mars {
  namespace plugins {
    namespace test {

      using namespace mars::utils;
      using namespace mars::interfaces;

      Test::Test(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "Test") {
      }
  
      void Test::init() {
	  envire::core::Item<mars::interfaces::ItemManagerInterface*> itemTest;        
      control->items->test();
      itemTest.setData(control->items);

      }

      void Test::reset() {
      }

      Test::~Test() {
      }


      void Test::update(sReal time_ms) {

        // control->motors->setMotorValue(id, value);
      }

      void Test::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
      }
  
      void Test::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

    } // end of namespace test
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test::Test);
CREATE_LIB(mars::plugins::test::Test);
