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
        TreeMars* treeMars = new TreeMars(control);
        TreeMarsInterface* treeMarsInterface = treeMars;
        treeMarsInterface -> minimalTest();
        treeMarsInterface -> test();
      }

      void TestTreeMars::reset() {
      }

      TestTreeMars::~TestTreeMars() {
      }


      void TestTreeMars::update(sReal time_ms) {

        // control->motors->setMotorValue(id, value);
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
