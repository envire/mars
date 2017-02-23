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

#ifndef MARS_INTERFACES_CONTACT_PARAMS_H
#define MARS_INTERFACES_CONTACT_PARAMS_H

#include "MARSDefs.h"
#include <mars/utils/Vector.h>
#include <smurf/ContactParams.hpp>

namespace mars {
  namespace interfaces {

    struct contact_params {
      void setZero(){
        max_num_contacts = 4;
        erp = 0.1;
        cfm = 0.00000001;
        friction1 = 0.8;
        friction2 = 0.8;
        friction_direction1 = 0;
        motion1 = motion2 = 0;
        fds1 = fds2 = 0;
        bounce = bounce_vel = 0;
        approx_pyramid = 1;
        coll_bitmask = 65535;
        depth_correction = 0.0;
      }

      contact_params(){
        setZero();
      }

      void fromSmurfCP(const smurf::ContactParams& smurf_cp) {
        max_num_contacts = smurf_cp.max_num_contacts;
        erp = smurf_cp.erp;
        cfm = smurf_cp.cfm;
        friction1 = smurf_cp.friction1;
        friction2 = smurf_cp.friction2;
        friction_direction1 = smurf_cp.friction_direction1;
        motion1 = smurf_cp.motion1;
        motion2 = smurf_cp.motion2;
        fds1 = smurf_cp.fds1;
        fds2 = smurf_cp.fds2;
        bounce = smurf_cp.bounce;
        bounce_vel = smurf_cp.bounce_vel;
        approx_pyramid = smurf_cp.approx_pyramid;
        coll_bitmask = smurf_cp.coll_bitmask;
        depth_correction = smurf_cp.depth_correction;
      }

      int max_num_contacts;
      sReal erp, cfm;
      sReal friction1, friction2;
      utils::Vector *friction_direction1;
      sReal motion1, motion2;
      sReal fds1, fds2;
      sReal bounce, bounce_vel;
      bool approx_pyramid;
      int coll_bitmask;
      sReal depth_correction;
    }; // end of struct contact_params

  } // end of namespace interfaces
} // end of namespace mars

#endif /* MARS_INTERFACES_CONTACT_PARAMS_H */
