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
 * \file SMURFToSimulation.cpp
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Tests
 *
 * Version 0.1
 */

#include "SMURFToSimulation.h"

#include <mars/interfaces/Logging.hpp>

#include <lib_manager/LibManager.hpp>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/LoadCenter.h>

#include <lib_config/YAMLConfiguration.hpp>
// To populate the Graph from the smurf
#include <smurf/Robot.hpp>
#include <envire_smurf/GraphLoader.hpp>

// For the floor
#include <mars/interfaces/NodeData.h>
#include <mars/sim/ConfigMapItem.h>

// To log the graph
#include <base/Time.hpp>
#include <envire_core/graph/GraphViz.hpp>
#include <envire_core/graph/EnvireGraph.hpp>

using vertex_descriptor = envire::core::GraphTraits::vertex_descriptor;

namespace mars {
    namespace plugins {
        namespace SMURFToSimulation {
            
            using namespace mars::utils;
            using namespace mars::interfaces;
            
            SMURFToSimulation::SMURFToSimulation(lib_manager::LibManager *theManager)
                : LoadSceneInterface(theManager), control(NULL), nextGroupId(1)
            {
                mars::interfaces::SimulatorInterface *marsSim;
                marsSim = libManager->getLibraryAs<mars::interfaces::SimulatorInterface>("mars_sim");
                if(marsSim) {
                    control = marsSim->getControlCenter();
                    control->loadCenter->loadScene[".zsmurf"] = this; // zipped smurf model
                    control->loadCenter->loadScene[".zsmurfs"] = this; // zipped smurf scene
                    control->loadCenter->loadScene[".smurf"] = this; // smurf model
                    control->loadCenter->loadScene[".smurfs"] = this; // smurf scene
                    control->loadCenter->loadScene[".svg"] = this; // smurfed vector graphic
                    control->loadCenter->loadScene[".urdf"] = this; // urdf model
                    LOG_INFO("smurftosimulation_loader: added SMURF loader to loadCenter");                    
                }
            }

            SMURFToSimulation::~SMURFToSimulation() {
              if(control) {
                control->loadCenter->loadScene.erase(".zsmurf");
                control->loadCenter->loadScene.erase(".zsmurfs");
                control->loadCenter->loadScene.erase(".smurf");
                control->loadCenter->loadScene.erase(".smurfs");
                control->loadCenter->loadScene.erase(".svg");
                control->loadCenter->loadScene.erase(".urdf");
                libManager->releaseLibrary("mars_sim");
              }                
            }       

            bool SMURFToSimulation::loadFile(std::string filename, std::string tmpPath,
                                    std::string robotname)
            {
                LOG_INFO("smurftosimulation_loader: load File");   

                vertex_descriptor center = addCenter();

                std::string path = libConfig::YAMLConfigParser::applyStringVariableInsertions(filename); 
                LOG_DEBUG("Robot Path: %s",  path.c_str() );
                envire::core::Transform iniPose;
                iniPose.transform.orientation = base::Quaterniond::Identity();
                iniPose.transform.translation << 1.0, 1.0, 0.3;
                smurf::Robot* robot = new( smurf::Robot);
                robot->loadFromSmurf(path);
                envire::smurf::GraphLoader graphLoader(control->graph);
                graphLoader.loadRobot(nextGroupId, center, iniPose, *robot);
                LOG_INFO("smurftosimulation_loader: load finished");   
            }

            int SMURFToSimulation::saveFile(std::string filename, std::string tmpPath)
            {
                return 0;
            }
            
            vertex_descriptor SMURFToSimulation::addCenter()
            {
                envire::core::FrameId center = "center";
                control->graph->addFrame(center);
                return control->graph->getVertex(center);
            }

            void SMURFToSimulation::addFloor(const vertex_descriptor &center)
            {
                NodeData data;
                data.init("floorData", Vector(0,0,0));
                data.initPrimitive(interfaces::NODE_TYPE_BOX, Vector(25, 25, 0.1), 0.1);
                data.movable = false;
                mars::sim::PhysicsConfigMapItem::Ptr item(new mars::sim::PhysicsConfigMapItem);
                data.material.transparency = 0.5;
                //data.material.ambientFront = mars::utils::Color(0.0, 1.0, 0.0, 1.0);
                // TODO Fix the material data is lost in the conversion from/to configmap
                data.material.emissionFront = mars::utils::Color(1.0, 1.0, 1.0, 1.0);
                LOG_DEBUG("Color of the Item in the addFloor: %f , %f, %f, %f", data.material.emissionFront.a , data.material.emissionFront.b, data.material.emissionFront.g, data.material.emissionFront.r );
                data.toConfigMap(&(item.get()->getData()));
                control->graph->addItemToFrame(control->graph->getFrameId(center), item);
            }
            
            void SMURFToSimulation::addRobot(vertex_descriptor center, const std::string& smurf_path)
            {    
                std::string path = libConfig::YAMLConfigParser::applyStringVariableInsertions(smurf_path); 
                LOG_DEBUG("Robot Path: %s",  path.c_str() );
                envire::core::Transform iniPose;
                iniPose.transform.orientation = base::Quaterniond::Identity();
                iniPose.transform.translation << 1.0, 1.0, 0.3;
                smurf::Robot* robot = new( smurf::Robot);
                robot->loadFromSmurf(path);
                envire::smurf::GraphLoader graphLoader(control->graph);
                graphLoader.loadRobot(nextGroupId, center, iniPose, *robot);
            }                                             
        } // end of namespace SMURFToSimulation
    } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::SMURFToSimulation::SMURFToSimulation);
CREATE_LIB(mars::plugins::SMURFToSimulation::SMURFToSimulation);