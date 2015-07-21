/*
 *  Copyright 2015, DFKI GmbH Robotics Innovation Center
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
 * \file EnvireMarsTree.h
 * \author Raul Dominguez 
 * \brief "EnvireMarsTree" is using the envire core tree EnvireTree to store the information about the Nodes or Items in the simulation
 *
 *
 * ToDo:
 *
 *      - First task: Test using a simple item that is a flat surface for
 *      the ground. The item is part of a EnvireMarsTree and is loaded with
 *      its particular plugin. It will interact with the Wordlphysics class
 *      and will be visible using the Envire Visualization tools, but not
 *      those from Mars.
 *
 */

#ifndef ENVIRE_MARS_TREE_H
#define ENVIRE_MARS_TREE_H

#ifdef _PRINT_HEADER_
  #warning "EnvireMarsTree.h"
#endif

#include "WorldPhysics.h"
#include "envire_core/TransformTree.hpp"

namespace mars {
  namespace sim {
    class EnvireMarsTree : Envire::Core::TransformTree{
    };
  } // end of namespace sim
} // end of namespace mars

#endif  // ENVIRE_MARS_TREE_H
