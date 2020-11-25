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
 * \file EnvireGraphLoader.cpp
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Tests
 *
 * Version 0.1
 */

#include "EnvireGraphLoader.hpp"

#include <mars/interfaces/Logging.hpp>

#include <lib_manager/LibManager.hpp>

#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/LoadCenter.h>

#include <mars/sim/PhysicsMapper.h>
#include <mars/sim/defines.hpp>

#include <mars/utils/misc.h>

// To log the graph
#include <base/Time.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>

#include <mars/utils/Vector.h>

#define MLS_FRAME_TF_X 0.0
#define MLS_FRAME_TF_Y 0.0
#define MLS_FRAME_TF_Z 0.0
#define MLS_FRAME_TF_ROT_X 0.0 

using vertex_descriptor = envire::core::GraphTraits::vertex_descriptor;

namespace mars {
    namespace plugins {
        namespace EnvireGraphLoader {
            
            using namespace mars::utils;
            using namespace mars::interfaces;
            
            EnvireGraphLoader::EnvireGraphLoader(lib_manager::LibManager *theManager)
                : LoadSceneInterface(theManager), control(NULL)
            {
                mars::interfaces::SimulatorInterface *marsSim;
                marsSim = libManager->getLibraryAs<mars::interfaces::SimulatorInterface>("mars_sim");
                if(marsSim) {
                    control = marsSim->getControlCenter();
                    control->loadCenter->loadScene[".graph"] = this; // urdf model
                    LOG_INFO("[envire_graph_loader::EnvireGraphLoader] Graph loader register to loadCenter");                    
                }

                center = SIM_CENTER_FRAME_NAME;
                mlsFrameId = MLS_FRAME_NAME; 
            }

            EnvireGraphLoader::~EnvireGraphLoader() {
              if(control) {
                control->loadCenter->loadScene.erase(".graph");
                libManager->releaseLibrary("mars_sim");
              }                
            }       

            bool EnvireGraphLoader::loadFile(std::string filename, std::string tmpPath,
                                    std::string robotname)
            {
                LOG_INFO("[envire_graph_loader::loadFile] This method is just a Dummy version of loadFile with pos and rot");                    
                utils::Vector pos(0.0,0.0,0.0);
                utils::Vector rot(0.0,0.0,0.0);
                LOG_INFO("[envire_graph_loader::loadFile] About to call the alternative loadFile");                    
                bool ok = loadFile(filename, tmpPath, robotname, pos, rot);
                return ok;
            }

            bool EnvireGraphLoader::loadFile(std::string filename, std::string tmpPath,
                                std::string robotname, utils::Vector pos, utils::Vector rot)
            {
                // TODO: use the pos and rot received from the configuration or
                // the ones of the original graph instead of the hardcoded ones
                LOG_DEBUG("[EnvireGraphLoader::loadFile] Graph loader given position");
                std::string suffix = utils::getFilenameSuffix(filename);
                if (! control->graph->containsFrame(center))
                {
                  control->graph->addFrame(center);
                }
                if (! control->graph->containsFrame(mlsFrameId))
                {
                  control->graph->addFrame(mlsFrameId);
                }
                // Create the default frame for the MLS but leave it empty.
                // The mls is loaded in the first update.
                envire::core::Transform mlsTf(base::Time::now());
                mlsTf.transform.translation << double(MLS_FRAME_TF_X), double(MLS_FRAME_TF_Y), double(MLS_FRAME_TF_Z);
                mlsTf.transform.orientation = base::AngleAxisd(double(rot[2]), base::Vector3d::UnitZ()); //Rotation for now only on Z
                control->graph->addTransform(center, mlsFrameId, mlsTf);
                loadMLSMap(filename);
                return true;
            }    

            int EnvireGraphLoader::saveFile(std::string filename, std::string tmpPath)
            {
                return 0;
            }

            void EnvireGraphLoader::loadMLSMap(const std::string & mlsPath)
            {
                /* Loads in the envire graph the mls given in the path after
                 * deserializing it.
                 *
                 * The serialized object is graph containing the mls in DUMPED_MLS_FRAME
                 */
                envire::core::EnvireGraph auxMlsGraph;
                auxMlsGraph.loadFromFile(mlsPath);
                envire::core::FrameId dumpedFrameId(DUMPED_MLS_FRAME_NAME);
                mlsPrec mlsAux = getMLSMap(auxMlsGraph, dumpedFrameId);
                envire::core::Item<mlsPrec>::Ptr mlsItemPtr(new envire::core::Item<mlsPrec>(mlsAux));
                control->graph->addItemToFrame(mlsFrameId, mlsItemPtr);
            }

            mlsPrec EnvireGraphLoader::getMLSMap(const envire::core::EnvireGraph & graph, envire::core::FrameId frameId)
            {
                mlsPrec mls;
                if (graph.containsItems<envire::core::Item<mlsKal>>(frameId))
                {
                    // The serialized Mls is not in Precalculated but in Kalman, so we have to convert it
                    envire::core::EnvireGraph::ItemIterator<envire::core::Item<mlsKal>> beginItem, endItem;
                    boost::tie(beginItem, endItem) = graph.getItems<envire::core::Item<mlsKal>>(frameId);
                    mls = beginItem->getData(); // Here the conversion to Precalculated occurs (mlsKal->mlsPrec)
                }
                else if (graph.containsItems<envire::core::Item<mlsPrec>>(frameId))
                {
                    envire::core::EnvireGraph::ItemIterator<envire::core::Item<mlsPrec>> beginItem, endItem;
                    boost::tie(beginItem, endItem) = graph.getItems<envire::core::Item<mlsPrec>>(frameId);
                    mls = beginItem->getData(); 
                }
                return mls;
            }

        } // end of namespace EnvireGraphLoader
    } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::EnvireGraphLoader::EnvireGraphLoader);
CREATE_LIB(mars::plugins::EnvireGraphLoader::EnvireGraphLoader);
