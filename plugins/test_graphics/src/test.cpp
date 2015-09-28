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
#include <envire_core/Transform.hpp>
#include <envire_core/TransformGraph.hpp>
#include <envire_core/events/ItemAddedEvent.hpp>
#include <envire_core/events/TransformAddedEvent.hpp>
#include <envire_core/TransformGraphExceptions.hpp>
#include <mars/sim/ConfigMapItem.h>
#include <base/TransformWithCovariance.hpp>
#include <stdlib.h>
#include <iostream>

namespace mars {
  namespace plugins {
    namespace test_graphics {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace envire::core;
      using namespace mars::sim;
      using namespace std;

      Test::Test(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "Test"), originId(""){
      }
  
      void Test::init() {
        
        if(control->graph != nullptr)
        {
          control->graph->subscribe(std::shared_ptr<GraphEventDispatcher>(this));
        }
      }

      void Test::reset() {
      }

      Test::~Test() {
        if(control->graph != nullptr)
        {
          control->graph->unsubscribe(std::shared_ptr<GraphEventDispatcher>(this));
        }
      }

      void Test::transformAdded(const envire::core::TransformAddedEvent& e)
      {
        if(originId.empty())
        {
          //use the first frame we get as originId
          originId = e.origin;
        }
      }
      
      void Test::transformRemoved(const envire::core::TransformRemovedEvent& e)
      {
        //TODO figure out what to do when the origin frame gets removed
      }
      
      void Test::transformModified(const envire::core::TransformModifiedEvent& e)
      {
      }
      
      void Test::itemAdded(const envire::core::ItemAddedEvent& e)
      {
        boost::intrusive_ptr<ConfigMapItem> pItem;
        if(pItem = boost::dynamic_pointer_cast<ConfigMapItem>(e.item))
        {
          try
          {         
            NodeData node;
            node.fromConfigMap(&pItem->getData(), "");
            Transform fromOrigin = control->graph->getTransform(originId, e.frame); 
            node.pos = fromOrigin.transform.translation;
            node.rot = fromOrigin.transform.orientation;
            const int drawId = control->graphics->addDrawObject(node);
            //TODO remeber the drawId
          }
          catch(const UnknownTransformException& ex)
          {
            cerr << ex.what() << endl;
          }
        }
      }
      
      

      void Test::update(sReal time_ms) {

      }

      void Test::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
      }

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test_graphics::Test);
CREATE_LIB(mars::plugins::test_graphics::Test);
