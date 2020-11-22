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
 * \file EnvireMls.cpp
 * \author Raul (Raul.Dominguez@dfki.de)
 * \brief Provides
 *
 * Version 0.1
 */

#include "EnvireMls.hpp"

#include <fstream>
#include <boost/archive/binary_iarchive.hpp>
#include <envire_core/items/Transform.hpp>


#include <envire_core/graph/GraphDrawing.hpp>

#include <mars/sim/NodePhysics.h>

#include <mars/interfaces/sim/LoadCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <base/samples/RigidBodyState.hpp>
#include <mars/sim/defines.hpp>
#include <mars/sim/SimMotor.h>

#include <base/TransformWithCovariance.hpp>

//#define DEBUG 0

/*
This plugin requires the plugins envire_graph_loader and envire_smurf_loader
to load the mls and the rover respectively
*/

namespace mars {
  namespace plugins {
    namespace envire_mls {

      using namespace mars::utils;
      using namespace mars::interfaces;

      using namespace envire::core;
      using namespace maps::grid;

      //const TEST_MLS_PATH std::string("/simulation/mars/plugins/envire_mls/testMlsData/crater_simulation_mls.graph")
      //const TEST_MLS_PATH std::string("/simulation/mars/plugins/envire_mls/testMlsData/mls_map-cave-20171109.graph")
      //const TEST_MLS_PATH std::string("/simulation/mars/plugins/envire_mls/testMlsData/mls_map-cave-20171110.graph")
      const std::string TEST_MLS_PATH = "/simulation/mars/plugins/envire_mls/testMlsData/precalculated_mls_map-20171112.graph";
      const std::string MLS_NAME = "CavePrecalculated";
      const Vector MLS_FRAME_POS  = {0.0, 0.0, 0.0};
      const double MLS_FRAME_ROT_Z (M_PI*0.5);
      // Robot name is defined in the defines.hpp
      const Vector ROBOT_FRAME_POS = {0,0,0};
      const double ROBOT_FRAME_ROT_Z = 0.0;
      const std::vector<std::string> MOTOR_NAMES{"wheel_front_left_motor", "wheel_front_right_motor", "wheel_rear_left_motor", "wheel_rear_right_motor"};
      const bool MOVE_FORWARD=true;
      const double SPEED=0.5; // Rads per second . Wheels have 20cm rad, therefore speed for 2.5r/s is 0.5m/s, 0.5r/s is 0.1 m/s
      const bool LOAD_PLY=false;
      const std::string PLY_FILE = "/simulation/mars/plugins/envire_mls/testPLYData/pointcloud-20171110-2230.ply";

      EnvireMls::EnvireMls(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "EnvireMls") 
      {
        mlsCollision = envire::collision::MLSCollision::getInstance();
      }

      void EnvireMls::init() 
      {
#ifdef DEBUG
        LOG_DEBUG( "[EnvireMls::init] Tests"); 
#endif
#ifndef SIM_CENTER_FRAME_NAME
        LOG_DEBUG( "[EnvireMls::init] SIM_CENTER_FRAME_NAME is not defined "); 
#endif
#ifdef SIM_CENTER_FRAME_NAME
        LOG_DEBUG( "[EnvireMls::init] SIM_CENTER_FRAME_NAME is defined: " + SIM_CENTER_FRAME_NAME); 
#endif
#ifdef DEBUG
        LOG_DEBUG("[EnvireMls::init] mlsTf x y z %f, %f, %f", 
            mlsTf.transform.translation.x(), 
            mlsTf.transform.translation.y(), 
            mlsTf.transform.translation.z());
        LOG_DEBUG("[EnvireMls::init] Constants for x y z %f, %f, %f", 
            double(MLS_FRAME_TF_X), 
            double(MLS_FRAME_TF_Y), 
            double(MLS_FRAME_TF_Z));
#endif
        sceneLoaded = false;
        movingForward = false;
      }

      void EnvireMls::reset() { }

      EnvireMls::~EnvireMls() { }

      void EnvireMls::update(sReal time_ms) 
      {
        if (not sceneLoaded){
          loadMLSAndRobot();
          sceneLoaded = true;
        }
        else{
            if ((MOVE_FORWARD) && (!movingForward)){
                moveForwards();    
            }
        }
      }

      void EnvireMls::loadMLSMap()
      {
        if (LOAD_PLY)
        {
            loadSlopedFromPLY(); //old and untested in newer versions
        }
        else
        {
            Vector mlsRot(0,0,MLS_FRAME_ROT_Z);
            control->sim->loadScene(std::getenv(ENV_AUTOPROJ_ROOT) + TEST_MLS_PATH, MLS_NAME, MLS_FRAME_POS, mlsRot);
        }
      }

      void EnvireMls::loadRobot()
      {
        Vector robotRot(0,0,ROBOT_FRAME_ROT_Z);
        control->sim->loadScene(std::getenv(ENV_AUTOPROJ_ROOT) + ASGUARD_PATH, ROBOT_NAME, ROBOT_FRAME_POS, robotRot);
      }

      void EnvireMls::loadMLSAndRobot()
      {
#ifdef DEBUG
        std::string path = std::getenv(ENV_AUTOPROJ_ROOT) + TEST_MLS_PATH;
        LOG_DEBUG( "[EnvireMls::testAddMLSAndRobot] Mls to test with: " + path); 
#endif
        loadMLSMap();
#ifdef DEBUG
        LOG_DEBUG( "[EnvireMls::testAddMLSAndRobot] 2"); 
#endif
        loadRobot();
      }

      void EnvireMls::moveForwards()
      {
          mars::sim::SimMotor* motor;
          for(auto it: MOTOR_NAMES) {
            motor = control->motors->getSimMotorByName(it);
#ifdef DEBUG
            LOG_DEBUG( "[EnvireMls::moveForwards] Motor " + it +" received"); 
#endif
            motor->setVelocity(SPEED);
#ifdef DEBUG
            LOG_DEBUG( "[EnvireMls::moveForwards] Motor " + it +" set velocity sent"); 
#endif 
          }
          movingForward = true;
      }

      void EnvireMls::loadSlopedFromPLY()
      {
#ifdef DEBUG
          LOG_DEBUG( "[EnvireMls::loadSlopedFromPLY] Start"); 
#endif
          pcl::PLYReader reader;
          pcl::PointCloud<pcl::PointXYZ> cloud;
          reader.read(std::getenv(ENV_AUTOPROJ_ROOT) + PLY_FILE, cloud);
#ifdef DEBUG
          LOG_DEBUG( "[EnvireMls::loadSlopedFromPLY] Cloud has been generated"); 
#endif  
          Vector2ui num_cells {2500, 2500};
          const Vector2d resolution {0.3, 0.3};
          const MLSConfig config; // Thickness, gapsize
          mlsSloped mlsS(num_cells, resolution, config);
          mlsS.getLocalFrame().translation() << 0.5*mlsS.getSize(), 0;
          base::TransformWithCovariance tf = base::TransformWithCovariance::Identity();
          tf.setCovariance(base::TransformWithCovariance::Covariance::Identity()*0.001);
          mlsS.mergePointCloud(cloud, tf);
          Item<mlsPrec>::Ptr mlsItemPtr(new Item<mlsPrec>(mlsS));
          control->graph->addItemToFrame(mlsFrameId, mlsItemPtr);
#ifdef DEBUG
          LOG_DEBUG( "[EnvireMls::loadSlopedFromPLY] MLS loaded to graph"); 
#endif  
      }

    } // end of namespace envire_mls
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_mls::EnvireMls);
CREATE_LIB(mars::plugins::envire_mls::EnvireMls);
