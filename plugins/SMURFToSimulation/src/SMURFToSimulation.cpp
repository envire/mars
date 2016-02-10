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

using vertex_descriptor = envire::core::GraphTraits::vertex_descriptor;

namespace mars {
    namespace plugins {
        namespace SMURFToSimulation {
            
            using namespace mars::utils;
            using namespace mars::interfaces;
            
            SMURFToSimulation::SMURFToSimulation(lib_manager::LibManager *theManager)
            : MarsPluginTemplate(theManager, "SMURFToSimulation") {
            }
            
            vertex_descriptor SMURFToSimulation::addFloor()
            {
                envire::core::FrameId center = "center";
                control->graph->addFrame(center);
                NodeData data;
                data.init("floorData", Vector(0,0,0));
                data.initPrimitive(interfaces::NODE_TYPE_BOX, Vector(5, 5, 0.1), 0.1);
                data.movable = false;
                mars::sim::PhysicsConfigMapItem::Ptr item(new mars::sim::PhysicsConfigMapItem);
                data.material.transparency = 0.5;
                //data.material.ambientFront = mars::utils::Color(0.0, 1.0, 0.0, 1.0);
                // TODO Fix the material data is lost in the conversion from/to configmap
                data.material.emissionFront = mars::utils::Color(1.0, 1.0, 1.0, 1.0);
                LOG_DEBUG("Color of the Item in the addFloor: %f , %f, %f, %f", data.material.emissionFront.a , data.material.emissionFront.b, data.material.emissionFront.g, data.material.emissionFront.r );
                data.toConfigMap(&(item.get()->getData()));
                control->graph->addItemToFrame(center, item);
                return control->graph->getVertex(center);
            }
            
            void SMURFToSimulation::addRobot(vertex_descriptor center)
            {    
                envire::core::Transform iniPose;
                iniPose.transform.orientation = base::Quaterniond::Identity();
                iniPose.transform.translation << 1.0, 1.0, 0.3;
                std::string path = libConfig::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(SPACECLIMBER)%>"); 
                //std::string path = libConfig::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/<%=ENV(ASGUARD4)%>"); 
                //std::string path = libConfig::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/tools/smurf/test/sample_smurfs/two_boxes_joined/smurf/two_boxes.smurf"); 
                //std::string path = libConfig::YAMLConfigParser::applyStringVariableInsertions("<%=ENV(AUTOPROJ_CURRENT_ROOT) %>/tools/smurf/test/sample_smurfs/two_boxes_joined/smurf/two_boxes_dynamic_joint.smurf"); 
                LOG_DEBUG("Robot Path: %s",  path.c_str() );
                envire::smurf::Robot asguard(iniPose, path);
                asguard.initGraph(*(control->graph), center);
                envire::core::GraphViz viz;
                std::string timestamp = base::Time::now().toString();
                std::string name = "initGraph" + timestamp + ".dot";
                viz.write(*(control->graph), name);
                asguard.loadLinks(*(control->graph), nextGroupId);
                //name = "loadLinks" + timestamp + ".dot";
                //viz.write(*(control->graph), name);
                asguard.loadCollidables(*(control->graph));
                asguard.loadInertials(*(control->graph));
                //name = "loadInertialAndCollidables" + timestamp + ".dot";
                //viz.write(*(control->graph), name);
                asguard.loadFixedJoints(*(control->graph));
                //name = "loadFixedJoints" + timestamp + ".dot";
                //viz.write(*(control->graph), name);
                asguard.loadDynamicJoints(*(control->graph));
                //name = "loadDynamicJoints" + timestamp + ".dot";
                //viz.write(*(control->graph), name);
                asguard.loadVisuals(*(control->graph));
                //name = "loadVisuals" + timestamp + ".dot";
                //viz.write(*(control->graph), name);
                //LOG_DEBUG("Loaded to Mars/Envire graph");
            }
            
            void SMURFToSimulation::init()
            {
                nextGroupId = 1;
                vertex_descriptor center = addFloor();
                //envire::core::GraphViz viz;
                //std::string timestamp = base::Time::now().toString();
                //std::string name = "justFloor" + timestamp + ".dot";
                //viz.write(*(control->graph), name);
                addRobot(center);
                // uncomment to print the graph
                //envire::core::GraphViz viz;
                //std::string timestamp = base::Time::now().toString();
                //std::string name = "smurfToSimulationInit" + timestamp + ".dot";
                //viz.write(*(control->graph), name);
                // Place the robot
                //envire::core::Transform robotPose;
                // Update Transform event handler is not implemented
                //robotPose.transform.translation << 2, 3, 1.0;
                //robotPose.transform.orientation = base::Quaterniond::Identity();
                //control->graph->updateTransform(center, robotRoot, robotPose);
                
                
                // Create a Simulated Robot from the information in the robot model
                // robot.simulate();
                // Create configMap
                //
                // Load a Robot
                // robot = envire_smurf.loadFromSMURF(Robot);
                //configmaps::ConfigMap* map = new configmaps::ConfigMap;
                //std::cout << "Create configMap" << std::endl;
                //// Update the configMap from a Smurf file
                //std::string path = std::string(std::getenv("AUTOPROJ_CURRENT_ROOT")) + "/models/robots/asguard_v4/smurf/";
                //std::string fileName = "asguard_v4.smurf"; 
                //std::cout << "Complete Part: " << path << fileName << std::endl;
                //bool expandURIs = false;
                //boost::shared_ptr<urdf::ModelInterface> modelInterface;
                //std::cout << "Shared Pointer to modelInterface" << std::endl;
                //modelInterface = parseFile(map, path, fileName, expandURIs);
                //std::cout << "parseFile executed" << std::endl;
                //// Make the tree load the model and configMap
                
                //control->tree->loadRobot(modelInterface, *map);
                //control->tree->drawDotFile("smurf.dot");
                //// Plot the Tree
                ///*
                //typedef configmaps::ConfigMap::const_iterator MapIterator;
                //typedef configmaps::ConfigVectorTemplate<configmaps::ConfigItem>::const_iterator VectorIterator;
                //for (MapIterator iter = map -> begin(); iter != map -> end(); iter++)
                //{
                //  cout << "Key: " << iter->first << endl;
                //  for (VectorIterator iterV = iter-> second.begin(); iterV != iter-> second.end(); iterV++)
                //  {
                //    cout << "Vector item: " << iterV -> toString() << endl;
                //  }
                //}
                
                //std::string path2 = (std::string)(map->operator[]("path"));
                //std::cout << "Path 2: " << path2 << std::endl;
                //std::string filename2 = (std::string)(map->operator[]("file"));
                //std::cout << "Filename 2: " << filename2 << std::endl;
                //sim::SimEntity* entity = smurf.createEntity(*map);
                //*/
                
                
            }
            
            void SMURFToSimulation::reset() {
            }
            
            SMURFToSimulation::~SMURFToSimulation() {
            }
            
            
            void SMURFToSimulation::update(sReal time_ms) {
                
                // control->motors->setMotorValue(id, value);
            }
            
            void SMURFToSimulation::receiveData(const data_broker::DataInfo& info,
                                                const data_broker::DataPackage& package,
                                                int id) {
                // package.get("force1/x", force);
                                                }
                                                
                                                void SMURFToSimulation::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
                                                    
                                                    if(_property.paramId == example.paramId) {
                                                        example.dValue = _property.dValue;
                                                    }
                                                }
                                                
        } // end of namespace SMURFToSimulation
    } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::SMURFToSimulation::SMURFToSimulation);
CREATE_LIB(mars::plugins::SMURFToSimulation::SMURFToSimulation);
