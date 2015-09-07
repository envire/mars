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
 * \file SMURF2MarsTree.cpp
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Tests
 *
 * Version 0.1
 */


#include "SMURF2MarsTree.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

//#include <boost/function.hpp> // For the share_ptr
#include <urdf_model/model.h>

#include <configmaps/ConfigData.h>
#include <smurf_parser/SMURFParser.h>


namespace mars {
  namespace plugins {
    namespace SMURF2MarsTree {

      using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace smurf_parser;

      SMURF2MarsTree::SMURF2MarsTree(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "SMURF2MarsTree"), smurf(theManager) {
      }

  
      void SMURF2MarsTree::init() {

        std::cout << "Init method start" << std::endl;
        // Create configMap
        //
        configmaps::ConfigMap* map = new configmaps::ConfigMap;
        std::cout << "Create configMap" << std::endl;
        // Update the configMap from a Smurf file
        std::string path = std::string(std::getenv("AUTOPROJ_CURRENT_ROOT")) + "/models/robots/asguard_v4/smurf/";
        std::string fileName = "asguard_v4.smurf"; 
        std::cout << "Complete Part: " << path << fileName << std::endl;
        bool expandURIs = false;
        boost::shared_ptr<urdf::ModelInterface> modelInterface;
        std::cout << "Shared Pointer to modelInterface" << std::endl;
        modelInterface = parseFile(map, path, fileName, expandURIs);
        std::cout << "parseFile executed" << std::endl;
        // Create a Tree
        mars::sim::TreeMars* tree = new mars::sim::TreeMars(control);
        std::cout << "Pointer to the Tree created" << std::endl;
        // Make the tree load the model and configMap

        tree -> loadRobot(modelInterface, *map);
        // Plot the Tree
        /*
        typedef configmaps::ConfigMap::const_iterator MapIterator;
        typedef configmaps::ConfigVectorTemplate<configmaps::ConfigItem>::const_iterator VectorIterator;
        for (MapIterator iter = map -> begin(); iter != map -> end(); iter++)
        {
          cout << "Key: " << iter->first << endl;
          for (VectorIterator iterV = iter-> second.begin(); iterV != iter-> second.end(); iterV++)
          {
            cout << "Vector item: " << iterV -> toString() << endl;
          }
        }

        std::string path2 = (std::string)(map->operator[]("path"));
        std::cout << "Path 2: " << path2 << std::endl;
        std::string filename2 = (std::string)(map->operator[]("file"));
        std::cout << "Filename 2: " << filename2 << std::endl;
        sim::SimEntity* entity = smurf.createEntity(*map);
        */


      }

      void SMURF2MarsTree::reset() {
      }

      SMURF2MarsTree::~SMURF2MarsTree() {
      }


      void SMURF2MarsTree::update(sReal time_ms) {

        // control->motors->setMotorValue(id, value);
      }

      void SMURF2MarsTree::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
      }
  
      void SMURF2MarsTree::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

    } // end of namespace SMURF2MarsTree
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::SMURF2MarsTree::SMURF2MarsTree);
CREATE_LIB(mars::plugins::SMURF2MarsTree::SMURF2MarsTree);
