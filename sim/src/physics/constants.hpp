#ifndef DEFINES_H
#define DEFINES_H

#include <mars/utils/Vector.h>

//#define DEBUG_WORLD_PHYSICS 1 // Comment in to have logs from the physics simulator controller
#define DRAW_MLS_CONTACTS 1 // Comment in to have logs from the physics simulator controller

namespace mars {
  namespace sim {

    namespace constants
    {

      const std::string SIM_CENTER_FRAME_NAME = "center";
      const std::string MLS_FRAME_NAME = "mls_01";

      const std::string ROBOT_NAME = "Asguard_v4";
      const std::string ROBOT_ROOT_LINK_NAME = "body";
      const std::string ASGUARD_PATH = "/models/robots/asguard_v4/smurf/asguard_v4.smurf";
      // This is the name of the mls frame in the serialized graph that can be loaded
      // by mars
      const std::string DUMPED_MLS_FRAME_NAME = "mls_map";

      const std::string ENV_AUTOPROJ_ROOT = "AUTOPROJ_CURRENT_ROOT";

    }
  }
}

#endif

