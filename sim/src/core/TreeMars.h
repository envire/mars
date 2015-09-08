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
 * \author Raul Dominguez
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
#include <string>

namespace mars {
  namespace sim {

    using namespace envire::core;
    using namespace interfaces;

    /**
     * The declaration of the ItemManager class.
     *
     */
    class TreeMars : public TransformTree, 
    public interfaces::TreeMarsInterface, 
    public interfaces::GraphicsUpdateInterface{
    
      public: 
        TreeMars(ControlCenter *c);
        virtual ~TreeMars(){}
        virtual int test();

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
                       const mars::interfaces::NodeData& data,
                       const envire::core::Transform& location);
      protected:

      private:
        interfaces::ControlCenter *control;
        int visual_rep;
    };

  } // NS sim
} // NS mars
#endif  // TREE_MARS_H
