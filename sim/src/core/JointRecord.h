#include <string>
#include <memory>
#include <mars/interfaces/sim/JointInterface.h>
#include <mars/sim/SimJoint.h>
#include <smurf/Joint.hpp>

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
