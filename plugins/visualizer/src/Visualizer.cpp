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
 * \file Visualizer.cpp
 * \author Raul__author__Arne (raul.dominguez@dfki.de)
 * \brief visualizer
 *
 * Version 0.1
 */


#include "Visualizer.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>


namespace mars {
  namespace plugins {
    namespace visualizer {

      using namespace mars::utils;
      using namespace mars::interfaces;

      Visualizer::Visualizer(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "Visualizer"), visualizerInitialized(false){
      }
  
      void Visualizer::init() {
        if (control->graphics) {
          control->graphics->addGraphicsUpdateInterface(this);
        }
      }
      
      void Visualizer::reset() { 
      }

      Visualizer::~Visualizer() {
      }

      void Visualizer::preGraphicsUpdate(void)
      {
        if(!visualizerInitialized)
        {
          window.displayGraph(control->graph, "center");
          window.show();
          visualizerInitialized = true;
        }
      }

      
      void Visualizer::update(sReal time_ms) {
        // control->motors->setMotorValue(id, value);

      }

      void Visualizer::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
      }
  
      void Visualizer::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

    } // end of namespace visualizer
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::visualizer::Visualizer);
CREATE_LIB(mars::plugins::visualizer::Visualizer);
