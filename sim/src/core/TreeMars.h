#ifndef TREE_MARS_H
#define TREE_MARS_H

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

/**
 * \file TreeMars.h
 * \author Raul Dominguez, Yong-Ho Yoo, Arne Böckmann
 *
 * The TreeMars class inherits from the EnvireTree which is used to represent
 * an environment. In this case the environment represented is the simulated
 * one.
 */

#ifdef _PRINT_HEADER_
  #warning "TreeMars.h"
#endif

#include <envire_core/TransformTree.hpp>
#include <mars/interfaces/graphics/GraphicsUpdateInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/TreeMarsInterface.h>
#include "ItemNodeData.h"
#include <string>
#include <memory>
#include <boost/smart_ptr/intrusive_ptr.hpp>

namespace mars {
  namespace sim {

    using namespace envire::core;
    using namespace interfaces;
    class SimNode;
    
    typedef std::map<interfaces::NodeId, SimNode*> NodeMap;
    
    class TreeMars : public TransformTree,     
					 public interfaces::TreeMarsInterface, 
					 public interfaces::GraphicsUpdateInterface{

      public: 
        TreeMars(ControlCenter *c);
        virtual ~TreeMars(){}

        //TODO There has to be a way to set multiple or no data elements

        /**Adds an object to the tree and adds a visual representation
         * (including physics) of the object to the mars scene graph.
         *
         * @param name The name of the frame that will be inserted into the tree
         * @param data The data that should be inserted into the frame
         * @param location Location of the object. I.e. transformation from the
         *                 root node to the object.
         */
        void addObject(const std::string& name,
                      const mars::interfaces::NodeData& node,
                      const envire::core::Transform& location);

        /**Updates the physical state of the simulated items and
         * their corresponding vertices.
         * I.e. updates the nodes values from the physical layer
         * and copies position and rotation over to the corresponding edge
         * in the tree.
         */
        void updateItemDynamics(sReal calc_ms, bool physics_thread);

        /**Updates the position and rotation of the graphics objects
         * corresponding to the nodes if the simulator is running in
         * graphics mode (i.e. control->graphics exists).
         * Otherwise does nothing.
         */
        virtual void preGraphicsUpdate();
      protected:

        //TODO maybe return graphics object as well?
        //TODO decide whether shared_ptr or unique_ptr should be used
        std::shared_ptr<SimNode> createSimNode(boost::intrusive_ptr<ItemNodeData> ind);


      interfaces::ControlCenter *control;
      
      NodeMap simNodes;          
      NodeMap simNodesDyn; //contains the moveable nodes
      int visual_rep;

      //ItemNodeData itemNodeData;

      interfaces::NodeId next_node_id;
      bool update_all_nodes;

      NodeMap nodesToUpdate;
        
        
    };

  } // NS sim
} // NS mars
#endif  // TREE_MARS_H
