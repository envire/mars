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

#include "../sensor_bases.h"
#include "../NodeData.h"
#include "../nodeState.h"

#include <mars/utils/Vector.h>
#include <mars/utils/Quaternion.h>

namespace mars {

  //namespace sim {
    //class SimNode;
  //};

  namespace interfaces {

    /**
     * \author Yong-Ho Yoo \n
     * \brief "TreeMarsInterface" declares the interfaces for all ItemOperations
     * that are used for the communication between the simulation modules.
     *
     */
    
     
    class TreeMarsInterface {
    public:
      virtual ~TreeMarsInterface() {}
      virtual int test() = 0;
      //int test;
      //
      // int addObject(std::map conf);

 
    };

  } // end of namespace interfaces
} // end of namespace mars

#endif  // TREE_MARS_INTERFACE_H
