#ifndef WORLD_PHYSICS_CONSTANTS_H
#define WORLD_PHYSICS_CONSTANTS_H

/*
 * Original values:
 *
const bool FAST_STEP = false;

const sReal WORLD_CFM = 1e-10;
const sReal WORLD_ERP = 0.1;

const util::Vector WORLD_GRAVITY = Vector(0.0, 0.0, -9.81);

const sReal GROUND_FRICTION = 20;
const sReal GROUND_CFM = 0.00000001;
const sReal GROUND_ERP = 0.1;

const sReal STEP_SIZE = 0.01;
 *
 */

#include <mars/utils/Vector.h>

namespace mars {
  namespace sim {

    using namespace utils;
    using namespace interfaces;

    namespace world_physics_constants
    {

      const bool FAST_STEP = false;

      const sReal WORLD_CFM = 1e-10;
      const sReal WORLD_ERP = 0.1;

      const utils::Vector WORLD_GRAVITY = Vector(0.0, 0.0, -9.81);

      const sReal GROUND_FRICTION = 20;
      const sReal GROUND_CFM = 1e-10;
      const sReal GROUND_ERP = 0.1;

      //const sReal STEP_SIZE = 0.01;
      //const sReal STEP_SIZE = 0.001;
      const sReal STEP_SIZE = 0.0001;

    }
  }
}

#endif
