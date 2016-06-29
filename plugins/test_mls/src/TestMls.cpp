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
 * \file TestMls.cpp
 * \author Yong-Ho (yong-ho.yoo@dfki.de)
 * \brief to
 *
 * Version 0.1
 */


#include "TestMls.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/interfaces/Logging.hpp>

#include <lib_config/YAMLConfiguration.hpp>
// For the floor
#include <mars/interfaces/NodeData.h>
#include <mars/sim/ConfigMapItem.h>
// To log the graph
#include <base/Time.hpp>
#include <envire_core/graph/GraphViz.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
	
#include <mars/sim/PhysicsMapper.h>


namespace mars {
  namespace plugins {
    namespace test_mls {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace envire::core;
      using namespace mars::sim;

      TestMls::TestMls(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "TestMls") {
			
		mls_collision = envire::collision::MLSCollision::getInstance();
      }
      

      envire::core::FrameId TestMls::createFrame()
      {
		envire::core::FrameId center = "center";
		control->graph->addFrame(center);
		return center;
      }
    
      void TestMls::addMLS(envire::core::FrameId center)
      {		
		  
		NodeData* node(new NodeData);
		node->init("mls_1", Vector(0,0,0));
		node->physicMode = interfaces::NODE_TYPE_MLS;
		
		std::string env_path("MLSMapKalman_waves.bin");
		node->env_path = env_path;
		
		std::ifstream input(node->env_path,  std::ios::binary);
		boost::archive::polymorphic_binary_iarchive  ia(input);
	
		ia >> mls_kalman;
        
	    mls = static_cast< boost::shared_ptr<maps::grid::MLSMapKalman> >(&mls_kalman);   
		node->g_mls = (void*)(mls_collision->createNewCollisionObject(mls));//_userdata);	

		Vector pos1(1,1,1);
		node->pos = pos1;
	
  		node->movable = false;	
  		 	
        Item<NodeData>::Ptr itemPtr(new Item<NodeData>(*node));
        control->graph->addItemToFrame(center, itemPtr);        
		
      }

      void TestMls::addSphere(envire::core::FrameId center)
      {
		NodeData data;
		data.init("sphere_1", Vector(0,0,3));
		data.initPrimitive(interfaces::NODE_TYPE_SPHERE, Vector(0.1, 0, 10), 0.1);
		data.movable = true;
		data.density = 0.1;
		data.mass = 1;
		
		Vector pos(3,0,3);
		
		data.pos = pos;
		mars::sim::PhysicsConfigMapItem::Ptr item(new mars::sim::PhysicsConfigMapItem);

		data.material.transparency = 0.5;
		data.material.ambientFront = mars::utils::Color(0.0, 1.0, 0.0, 1.0);
		// TODO Fix the material data is lost in the conversion from/to configmap
		data.material.emissionFront = mars::utils::Color(1.0, 1.0, 1.0, 1.0);
		LOG_DEBUG("Color of the Item in the addSphere: %f , %f, %f, %f", data.material.emissionFront.a , data.material.emissionFront.b, data.material.emissionFront.g, data.material.emissionFront.r );
		data.toConfigMap(&(item.get()->getData()));
		control->graph->addItemToFrame(center, item);
		
		
      }      
      
      
      void TestMls::init() {
                envire::core::FrameId center = createFrame();
                addMLS(center);                
             //   addSphere(center);
            
      }

      void TestMls::reset() {
      }

      TestMls::~TestMls() {
      }


      void TestMls::update(sReal time_ms) {

      }

      void TestMls::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
      }
  
      void TestMls::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

    } // end of namespace test_mls
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::test_mls::TestMls);
CREATE_LIB(mars::plugins::test_mls::TestMls);
