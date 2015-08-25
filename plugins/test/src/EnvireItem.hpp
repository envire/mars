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

#ifndef ENVIRE_ITEM_H
#define ENVIRE_ITEM_H

#ifdef _PRINT_HEADER_
  #warning "EnvireItem.hpp"
#endif


#include "envire_core/Item.hpp"

using namespace envire::core;

namespace envire{
  namespace core{

    class EnvireItem : public Item<mars::sim::ItemManager> {

      public: 
        template <typename Ts>                                                      
          EnvireItem(Ts&& args) : Item<mars::sim::ItemManager> (std::forward<Ts>(args)){}
    };

  } // end namespace envire-core
}

#endif  // ITEM_PHYSICS_H
