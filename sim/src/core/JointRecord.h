#include <string>
#include <memory>
#include <mars/interfaces/sim/JointInterface.h>
#include "SimJoint.h"

namespace mars{
  namespace sim{
    struct JointRecord
    {
      std::string name;
      std::shared_ptr<mars::interfaces::JointInterface> interface;
      std::shared_ptr<mars::sim::SimJoint> sim;
    };
  }
}
