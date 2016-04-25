/*
 *  Copyright 2011, 2012, DFKI GmbH Robotics Innovation Center
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

/*
 *  NodeCOMSensor.cpp
 *  QTVersion
 *
 *  Created by Malte Römmerann
 *
 */

#include "NodeCOMSensor.h"
#include "NodePhysics.h"
#include "SimNode.h"
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>

#include <cstdio>
#include <cstdlib>

namespace mars {
  namespace sim {

    using namespace utils;
    using namespace interfaces;

    BaseSensor* NodeCOMSensor::instanciate(ControlCenter *control,
                                           BaseConfig *config ){
  
      IDListConfig *cfg = dynamic_cast<IDListConfig*>(config);
      assert(cfg);
      return new NodeCOMSensor(control,*cfg);
    }

    NodeCOMSensor::NodeCOMSensor(ControlCenter* control, IDListConfig config):
      NodeArraySensor(control, config, false, false) {
      typeName = "NodeCOM";
      frame = config.frame;
    }

    Vector NodeCOMSensor::getCenterOfMass() const{
      std::vector<NodeId>::const_iterator iter;
      std::vector<NodeInterface*> physicsNodesPtrs;
      using SimNodeItem = envire::core::Item<std::shared_ptr<SimNode>>;
      using Iterator = envire::core::EnvireGraph::ItemIterator<SimNodeItem>;
      std::shared_ptr<SimNode> simNodePtr;
      Iterator begin, end;
      boost::tie(begin, end) = control->graph->getItems<SimNodeItem>(frame);      
      if (begin != end){
        // NOTE config.ids seems  not to be started anywhere... by now all simNodes are pused in the vector
        simNodePtr = begin->getData(); 
        //for (iter = config.ids.begin(); iter != config.ids.end(); iter++) {
        //  LOG_DEBUG("[NodeCOMSensor::getCenterOfMass] simNodePtr ID %d", simNodePtr->getID());
        //  LOG_DEBUG("[NodeCOMSensor::getCenterOfMass] Looking for %d", (*iter));
        //  if (simNodePtr->getID()==(*iter))
        //  {
        //    physicsNodesPtrs.push_back(simNodePtr->getInterface());
        //  }
        //}
        physicsNodesPtrs.push_back(simNodePtr->getInterface());
      }
      Vector center = control->sim->getPhysics()->getCenterOfMass(physicsNodesPtrs);
      return center;
    }
    
    
    // this function should be overwritten by the special sensor to
    int NodeCOMSensor::getAsciiData(char* data) const {
      Vector center = getCenterOfMass();
      sprintf(data, " %6.2f %6.2f %6.2f", center.x(), center.y(), center.z());
      return 21;
    }

    int NodeCOMSensor::getSensorData(sReal** data) const {
      // NOTE In the current implementation, there could be more than one node physics in the frame. Can we restrict this? If they are used for the joints, why do we need more than one? Can't we have a same physics node being joined to multiple?
      
      // NOTE This is not working if the sensors are in various frames (usual case). For the case of CREX we only have one, therefore it will work fine.
      // FIXME This method should look accross all the frames not only in the one of the attribute frame
      
      Vector center = getCenterOfMass();
  
      *data = (sReal*)calloc(3, sizeof(sReal));
      (*data)[0] = center.x();
      (*data)[1] = center.y();
      (*data)[2] = center.z();
      return 3;
    }

  } // end of namespace sim
} // end of namespace mars
