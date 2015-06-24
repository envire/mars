#ifndef ENVIRE_MARS_H
#define ENVIRE_MARS_H

#ifdef _PRINT_HEADER_
  #warning "EnvireMars.h"
#endif

namespace Mars {
  namespace envire_mars {

    class SimJoint;
    class SimNode;

    typedef std::map<interfaces::NodeId, SimNode*> NodeMap;

    /**
     * The declaration of the NodeManager class.
     *
     */
    class EnvireMars : public interfaces::EnvireMarsInterface {
    public:
      EnvireMars(interfaces::ControlCenter *c);
      virtual ~EnvireMars(){}


      virtual interfaces::NodeId addNode(interfaces::NodeData *nodeS,
                                         bool reload = false,
                                         bool loadGraphics = true);

    private:

      NodeMap simNodes;
      //NodeMap simNodesDyn;
      //NodeMap nodesToUpdate;
      //std::list<interfaces::NodeData> simNodesReload;
    };

  } // end of namespace envire_mars
} // end of namespace mars

#endif  // NODE_MANAGER_H
