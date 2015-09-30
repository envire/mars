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
    namespace test_graph {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace envire::core;
      using namespace mars::sim;
      using namespace std;

      TestGraph::TestGraph(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "TestGraph"){
      }
  
      void TestGraph::init() 
      {
        FrameId originId = "origin";
        NodeType type;
        for(int i = 0; i < 42; ++i)
        {
          const string name = "item " + boost::lexical_cast<string>(i);
          
          const int r = rand() % 4;
          switch(r)
          {
            case 0: type = interfaces::NODE_TYPE_BOX; break;
            case 1: type = interfaces::NODE_TYPE_SPHERE; break;
            case 2: type = interfaces::NODE_TYPE_CAPSULE; break;
            case 3: type = interfaces::NODE_TYPE_CYLINDER; break;
            default: type = interfaces::NODE_TYPE_BOX; break;
          }
          Transform tf;
          tf.transform.translation << rand() % 20, rand() % 20, rand() % 20;
          base::Vector3d axis;
          axis.setRandom();
          tf.transform.orientation = base::AngleAxisd(double(rand()) / (RAND_MAX/2), axis);
          
          NodeData data;
          data.init(name, Vector(0,0,0));
          data.initPrimitive(type, Vector(0.2, 0.2, 0.2), 0.1);
          data.movable = false;
          data.material.transparency = 0.5;
          boost::intrusive_ptr<ConfigMapItem> item(new ConfigMapItem);
          data.toConfigMap(&(item.get()->getData()));
          control->graph->addTransform(originId, name, tf);
          transforms.emplace_back(originId, name);
          control->graph->addItemToFrame(name, item);
        }
      }

      void TestGraph::reset() {
      }

      TestGraph::~TestGraph() {
      
      }

      
      

      void TestGraph::update(sReal time_ms) 
      {
        //rotate the objects
        for(const pair<FrameId, FrameId>& transform : transforms)
        {
          Transform tf = control->graph->getTransform(transform.first, transform.second);
          tf.transform.orientation *= base::Quaterniond(base::AngleAxisd(0.017, base::Vector3d(0, 1, 0)));
          control->graph->updateTransform(transform.first, transform.second, tf);
        }
      }

      void TestGraph::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
      }

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test_graph::TestGraph);
CREATE_LIB(mars::plugins::test_graph::TestGraph);
