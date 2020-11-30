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
#include <envire_core/graph/TreeView.hpp>
#include <yaml-cpp/yaml.h>

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
      //const Vector MLS_FRAME_POS  = {0.0, 0.0, 0.0};
      //const double MLS_FRAME_ROT_Z (M_PI*0.5);
      //const double MLS_FRAME_ROT_Z = 0.0;
      // Robot name is defined in the defines.hpp
      //const Vector ROBOT_FRAME_POS = {0,0,0};
      //const double ROBOT_FRAME_ROT_Z = 0.0;
      const std::vector<std::string> MOTOR_NAMES{"wheel_front_left_motor", "wheel_front_right_motor", "wheel_rear_left_motor", "wheel_rear_right_motor"};
      const bool MOVE_FORWARD=true;
      const double SPEED=0.5; // Rads per second . Wheels have 20cm rad, therefore speed for 2.5r/s is 0.5m/s, 0.5r/s is 0.1 m/s
      const bool LOAD_PLY=false;
      const std::string PLY_FILE = "/simulation/mars/plugins/envire_mls/testPLYData/pointcloud-20171110-2230.ply";
      const std::string yaml_path = "/simulation/mars/plugins/envire_mls/testMlsData/conf.yml";
      const std::vector<std::string> confs = {"/simulation/mars/plugins/envire_mls/testMlsData/conf.yml"};//,
                                              //"/simulation/mars/plugins/envire_mls/testMlsData/conf.yml"};

      EnvireMls::EnvireMls(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "EnvireMls") 
      {
        mlsCollision = envire::collision::MLSCollision::getInstance();
        robPos = {0.0, 0.0, 0.0};
        robOri = 0.0;
        mlsPos = {0.0, 0.0, 0.0};
        mlsOri = 0.0;
        mapStringParams["robPos"] = robPosP;
        mapStringParams["robOri"] = robOriP;
        mapStringParams["mlsPos"] = mlsPosP;
        mapStringParams["mlsOri"] = mlsOriP;
        mapStringParams["robGoal"] = robGoalP;
        numConfsExecuted = 0;
        confLoaded = false;
      }

      void EnvireMls::init() 
      {
        loadConf(yaml_path);
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

        if (control->graph->containsFrame(MLS_FRAME_NAME))
        {
#ifdef DEBUG
          envire::core::Transform tfMlsCen = control->graph->getTransform(MLS_FRAME_NAME, SIM_CENTER_FRAME_NAME);
          LOG_DEBUG("[EnvireMls::Update]: Transformation between MLS and center " + tfMlsCen.toString()); 
#endif
        }

        control->sim->StartSimulation();
      }

      void EnvireMls::reset() { }

      EnvireMls::~EnvireMls() { }

      void EnvireMls::update(sReal time_ms) 
      {
        // Go over the different confs for each start timer prepare the scene
        // let it run stop timer save output. Store also the initial distance.

        // States: No conf loaded, simulation running, all confs done
        bool allConfsDone = (numConfsExecuted == confs.size());
        if (!allConfsDone)
        {
          if (!confLoaded)
          {              
            confLoaded = loadConf(confs[numConfsExecuted]);
            LOG_DEBUG( "[EnvireMls::update] Configuration Loaded"); 
          }
          if (confLoaded)
          {
            if (!sceneLoaded)
            {
              sceneLoaded = loadMLSAndRobot();
              LOG_DEBUG( "[EnvireMls::update] Scene Loaded"); 
              // void NodeManager::clearAllNodes(bool clear_all, bool clearGraphics) ?
            }
            if (sceneLoaded)
            {
              if (goalReached())
              {
                //control->sim->StopSimulation();
                //LOG_DEBUG( "[EnvireMls::update] Simulation stopped"); 
                //cleanUp();
                numConfsExecuted ++;
                if (numConfsExecuted <= confs.size())
                {
                  //cleanUp();
                  confLoaded = false;
                  sceneLoaded = false;
                }
                LOG_DEBUG( "[EnvireMls::update] Goal was reached"); 
                //control->sim->exitMars();
              }
              else
              {
                if ((MOVE_FORWARD) && (!movingForward))
                {
                    moveForwards();    
                }
                //envire::core::FrameId robotFrame = ROBOT_ROOT_LINK_NAME;
                //envire::core::EnvireGraph copy(*control->graph.get());
                //bool robotFrameFound = (control->graph->containsFrame(robotFrame));
                //bool mlsFrameFound = (control->graph->containsFrame(MLS_FRAME_NAME));
                //bool robotFrameFoundCopy = (copy.containsFrame(robotFrame));
                //LOG_DEBUG( "[EnvireMls::update] RobotFrameFound %b", robotFrameFound ); 
                //LOG_DEBUG( "[EnvireMls::update] mlsFrameFound %b", mlsFrameFound); 
                //LOG_DEBUG( "[EnvireMls::update] RobotFrameFound Copy %b", robotFrameFoundCopy); 
              }
            }
          }
        }
        else // All done
        {
          control->sim->StopSimulation();
        }
      }

      void EnvireMls::cleanUp(){
        control->sim->StopSimulation();
        bool clear_all = true;
        bool clear_graphics = true;
        control->nodes->clearAllNodes(clear_all, clear_graphics);
        //control->graph.reset(new envire::core::EnvireGraph);
        //control->graph->addFrame(SIM_CENTER_FRAME_NAME);
        control->sim->StartSimulation();

        //envire::core::TreeView robotTree = control->graph->getTree(control->graph->getVertex(SIM_CENTER_FRAME_NAME));
        //LOG_DEBUG("robotTree root children size: %i", robotTree.tree[robotTree.root].children.size());
        //bool robotRemoved = (!control->graph->containsFrame(ROBOT_ROOT_LINK_NAME));
        //bool mlsRemoved = (!control->graph->containsFrame(MLS_FRAME_NAME));
        //// Use Simulator::NewWorld()
        //// Envire can't remove the next edge because on of them don't belong to the tree, but I think both should...
        //robotTree.removeEdge( 
        //  control->graph->getVertex(SIM_CENTER_FRAME_NAME),
        //  control->graph->getVertex(ROBOT_ROOT_LINK_NAME)
        //);
        //robotTree.removeEdge(
        //  control->graph->getVertex(SIM_CENTER_FRAME_NAME),
        //  control->graph->getVertex(MLS_FRAME_NAME)
        //);
        //LOG_DEBUG("After removing edges robotTree root children size: %i", robotTree.tree[robotTree.root].children.size());
        //robotRemoved = (!control->graph->containsFrame(ROBOT_ROOT_LINK_NAME));
        //mlsRemoved = (!control->graph->containsFrame(MLS_FRAME_NAME));
        
        //bool clearAll = true;
        //control->sim->newWorld(clearAll);

        //control->graph->removeTransform(SIM_CENTER_FRAME_NAME, ROBOT_ROOT_LINK_NAME);
        //control->graph->removeTransform(ROBOT_ROOT_LINK_NAME, SIM_CENTER_FRAME_NAME);
        //control->graph->removeFrame(ROBOT_ROOT_LINK_NAME);
        //control->graph->removeTransform(SIM_CENTER_FRAME_NAME, MLS_FRAME_NAME);
        //control->graph->removeTransform(MLS_FRAME_NAME, SIM_CENTER_FRAME_NAME);
        //control->graph->removeFrame(MLS_FRAME_NAME);
      }

      void EnvireMls::loadMLSMap()
      {
        if (LOAD_PLY)
        {
            loadSlopedFromPLY(); //old and untested in newer versions
        }
        else
        {
            Vector mlsRot(0,0,mlsOri);
            control->sim->loadScene(std::getenv(ENV_AUTOPROJ_ROOT) + TEST_MLS_PATH, MLS_NAME, mlsPos, mlsRot);
        }
      }

      void EnvireMls::loadRobot()
      {
        Vector robotRot(0,0,robOri);
        control->sim->loadScene(std::getenv(ENV_AUTOPROJ_ROOT) + ASGUARD_PATH, ROBOT_NAME, robPos, robotRot);
      }

      bool EnvireMls::loadMLSAndRobot()
      {
        bool loaded = false;
#ifdef DEBUG
        std::string path = std::getenv(ENV_AUTOPROJ_ROOT) + TEST_MLS_PATH;
        LOG_DEBUG( "[EnvireMls::testAddMLSAndRobot] Mls to test with: " + path); 
#endif
        loadMLSMap();
#ifdef DEBUG
        LOG_DEBUG( "[EnvireMls::testAddMLSAndRobot] 2"); 
#endif
        loadRobot();
        loaded = true;
        return loaded;
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

      bool EnvireMls::goalReached()
      {
        envire::core::Transform robPosTf = control->graph->getTransform(SIM_CENTER_FRAME_NAME, ROBOT_ROOT_LINK_NAME);
        base::Position robPos = robPosTf.transform.translation;
        Eigen::Vector3f v;
        Eigen::Vector3f w;
        v << robPos[0], robPos[1], robPos[2];
        w << robGoalPos[0], robGoalPos[1], robGoalPos[2];
        Eigen::Vector3f diff = v - w;
        float distance = diff.norm();
        bool reached = (distance <= 0.2);
        //LOG_DEBUG("[EnvireMls::goalReached] robPos x y z %f, %f, %f", 
        //    robPosTf.transform.translation.x(), 
        //    robPosTf.transform.translation.y(), 
        //    robPosTf.transform.translation.z());
        //LOG_DEBUG("[EnvireMls::goalReached] goalPos x y z %f, %f, %f", 
        //    robGoalPos[0], robGoalPos[1], robGoalPos[2]);
        //LOG_DEBUG("[EnvireMls::goalReached] distance %f", 
        //    distance);
        if (reached)
        {
          LOG_DEBUG( "[EnvireMls::goalReached] Target reached"); 
        }
        LOG_DEBUG( "[EnvireMls::goalReached] Distance: %f", distance); 
        return reached;
      }

      bool EnvireMls::yamlLoad(const std::string & confPath, YAML::Node & conf)
      {
        bool loaded = false;
        try{
          conf = YAML::LoadFile(std::getenv(ENV_AUTOPROJ_ROOT) + confPath);
          loaded = true;
        }catch (...){
          LOG_ERROR("[EnvireMls::init] Something went wrong loading the test config yaml."
          " It might have not been found. : %s", confPath.c_str());
        }
        return loaded;
      }

      bool EnvireMls::loadConf(const std::string & confPath)
      {
        bool loaded = false;
        YAML::Node conf;
        if(yamlLoad(confPath, conf))
        {
          for (const std::string& item: confItems)
          {
            LOG_INFO("[EnvireMls::init] confItem: " + item);
            if (conf[item])
            {
              switch (mapStringParams[item]){
                case robPosP:
                {
                  std::vector<double> robPosV = conf["robPos"].as<std::vector<double>>();
                  robPos = {robPosV[0], robPosV[1], robPosV[2]};
                  LOG_INFO(
                      "[EnvireMls::init] Loaded this position for the rover: %f, %f, %f ",
                      robPos[0], robPos[1], robPos[2]);
                  break;

                }
                case robOriP:
                {
                  robOri = conf["robOri"].as<double>();
                  LOG_INFO(
                      "[EnvireMls::init] Loaded orientation for the robot: %f ",
                      robOri);
                  break;
                }
                case mlsPosP:
                {
                  std::vector<double> mlsPosV = conf["mlsPos"].as<std::vector<double>>();
                  mlsPos = {mlsPosV[0], mlsPosV[1], mlsPosV[2]};
                  LOG_INFO(
                      "[EnvireMls::init] Loaded this position for the mls: %f, %f, %f",
                      mlsPos[0], mlsPos[1], mlsPos[2]);
                  break;
                }
                case mlsOriP:
                {
                  mlsOri = conf["mlsOri"].as<double>();
                  LOG_INFO(
                      "[EnvireMls::init] Loaded orientation for the mls: %f ",
                      mlsOri);
                  break;
                }
                case robGoalP:
                {
                  std::vector<double> robGoalV = conf["robGoal"].as<std::vector<double>>();
                  robGoalPos = {robGoalV[0], robGoalV[1], robGoalV[2]};
                  LOG_INFO(
                      "[EnvireMls::init] Loaded goal position: %f, %f, %f ",
                      robGoalPos[0], robGoalPos[1], robGoalPos[2]);
                  break;
                }
              }
            }
          }
          loaded = true;
        }
        return loaded;
      }

      bool EnvireMls::moveRobotTo(const envire::core::Transform& newPos)
      {
        return false;
      }

    } // end of namespace envire_mls
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_mls::EnvireMls);
CREATE_LIB(mars::plugins::envire_mls::EnvireMls);
