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
#include "envire_core/Transform.hpp"
#include <base/TransformWithCovariance.hpp>

#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/TreeMarsInterface.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/graphics/GraphicsUpdateInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>

#include <mars/utils/Mutex.h>

#include "SimNode.h"
#include "ItemNodeData.h"
#include "PhysicsMapper.h"
#include "SimEntity.h"

#include <string>
#include <memory>
#include <boost/smart_ptr/intrusive_ptr.hpp>

namespace urdf
{
  class ModelInterface;
}
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

        // Generate the nodes according to a SMURF Model
        virtual void loadRobot(boost::shared_ptr<urdf::ModelInterface> modelInterface,
                               const configmaps::ConfigMap & map);

        //TODO There has to be a way to set multiple or no data elements

        /**Adds an object to the tree and adds a visual representation
         * (including physics) of the object to the mars scene graph.
         *
         * @param name The name of the frame that will be inserted into the tree
         * @param data Describes what kind of item should be inserted into the
         *             simulation.
         *             Note: Position and orientation should be in the simulator
         *                   coordinate frame. I.e. in the root frame.
         * @param location Relative location of the object. I.e. transformation from the
         *                 parent node to the object.
         */
        NodeIdentifier addObject(const std::string& name,
                                 const mars::interfaces::NodeData& node,
                                 const envire::core::Transform& location,
                                 const NodeIdentifier& parent);

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

        //draws the current tree structure to a dot file */
        virtual void drawDotFile(const std::string& file) const;

        virtual NodeIdentifier getRoot() const;
      protected:

        //TODO maybe return graphics object as well?
        //TODO decide whether shared_ptr or unique_ptr should be used
        std::shared_ptr<SimNode> createSimNode(boost::intrusive_ptr<ItemNodeData> ind);

        void loadRobotRec(boost::shared_ptr<urdf::ModelInterface> modelInterface,
                          std::string startLinkName, std::vector<std::string>& visitedLinks,
                          bool root, NodeIdentifier parentNode, base::TransformWithCovariance rootToParent);

      private:
      interfaces::ControlCenter *control;
      mars::utils::Mutex mutex; /**<Used to lock access to the tree structure */
        
    };

  } // NS sim
} // NS mars
#endif  // TREE_MARS_H
