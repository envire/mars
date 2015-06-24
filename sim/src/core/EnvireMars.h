#ifndef ENVIRE_MARS_H
#define ENVIRE_MARS_H

#ifdef _PRINT_HEADER_
  #warning "EnvireMars.h"
#endif

namespace mars {
  namespace sim {

    typedef std::map<interfaces::NodeId, SimNode*> NodeMap;

    /**
     * The declaration of the NodeManager class.
     *
     */
    class EnvireMars  {
    public:
      EnvireMars(){}
      ~EnvireMars(){}

   // private:

      NodeMap simNodes;
      NodeMap simNodesDyn;
      NodeMap nodesToUpdate;
      std::list<interfaces::NodeData> simNodesReload;
      
      
    };

  } // end of namespace sim
} // end of namespace mars

#endif  // ENVIRE_MARS_H
