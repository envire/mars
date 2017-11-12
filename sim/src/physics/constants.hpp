#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>
#include <iostream>

//#define DEBUG_WORLD_PHYSICS 1 // Comment in to have logs from the physics simulator controller
#define DRAW_MLS_CONTACTS 1 // Comment in to have logs from the physics simulator controller

namespace mars {
  namespace sim {

    namespace constants
    {

      const char* SIM_CENTER_FRAME_NAME = "center";
      const char* MLS_FRAME_NAME = "mls_01";

      const char* ROBOT_NAME = "Asguard_v4";
      const char* ROBOT_ROOT_LINK_NAME = "body";
      const char* ASGUARD_PATH = "/models/robots/asguard_v4/smurf/asguard_v4.smurf";
      // This is the name of the mls frame in the serialized graph that can be loaded
      // by mars
      const char* DUMPED_MLS_FRAME_NAME = "mls_map";

      const char* ENV_AUTOPROJ_ROOT = "AUTOPROJ_CURRENT_ROOT";

    }
  }
}

#endif

