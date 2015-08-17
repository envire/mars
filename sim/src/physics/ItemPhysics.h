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
 * \file ItemPhysics.h
 * \author Raul Dominguez 
 * \brief "ItemPhysics" implements the physical ode stuff for the items that
 * are stored in the EnvireMarsTree.
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

#ifndef ITEM_PHYSICS_H
#define ITEM_PHYSICS_H

#ifdef _PRINT_HEADER_
  #warning "ItemPhysics.h"
#endif

//#include "WorldPhysics.h"
//#include "envire_core/TransformTree.hpp"
#include "envire_core/Item.hpp"
#include "NodePhysics.h"

using namespace envire::core;
using namespace std;

namespace mars {
  namespace sim {
    template <class T>
    class MarsItem : public envire::core::Item<T> {
      public:
        template <typename Ts>
          MarsItem(Ts&& args) : envire::core::Item<T> (std::forward<Ts>(args)) {}
        void hello();
    };
    class PhysicsItem : public MarsItem<mars::sim::NodePhysics> {
      public:
        template <typename Ts>
          PhysicsItem(Ts&& args) : MarsItem<mars::sim::NodePhysics> (std::forward<Ts>(args)) {}
    };
  }
}

#endif  // ITEM_PHYSICS_H
