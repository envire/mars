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
 * \file TreeMarsInterface.h
 * \author Yong-Ho Yoo \n
 * \brief "TreeMarsInterface" declares the interfaces for all ItemOperations
 * that are used for the communication between the simulation modules.
 *
 * \version 1.0
 * \date 20.08.2015
 */

#ifndef TREE_MARS_INTERFACE_H
#define TREE_MARS_INTERFACE_H

#ifdef _PRINT_HEADER_
  #warning "TreeMarsInterface.h"
#endif

#include <string>

namespace envire { namespace core
{ 
  class Transform; 
}}

namespace mars { namespace interfaces 
{
  class TreeMarsInterface {
  public:
    virtual ~TreeMarsInterface() {}
    virtual void addObject(const std::string& name,
                           const mars::interfaces::NodeData& node,
                           const envire::core::Transform& location) = 0;

    virtual void updateItemDynamics(sReal calc_ms, bool physics_thread) = 0;
};
}} // end of namespace mars::interfaces

#endif  // TREE_MARS_INTERFACE_H
