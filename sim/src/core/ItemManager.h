#ifndef ITEM_MANAGER_H
#define ITEM_MANAGER_H

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
 * \file ItemManager.h
 * \author Yong-Ho Yoo
 */

#ifdef _PRINT_HEADER_
  #warning "ItemManager.h"
#endif

#include <mars/utils/Mutex.h>
#include <mars/interfaces/graphics/GraphicsUpdateInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/ItemManagerInterface.h>
#include "ItemNodeData.h"

namespace mars {
  namespace sim {

    using namespace interfaces;
    class SimJoint;
    class SimNode;
    
    typedef std::map<interfaces::NodeId, SimNode*> NodeMap;
    /**
     * The declaration of the ItemManager class.
     *
     */
    class ItemManager : public interfaces::ItemManagerInterface,
                        public interfaces::GraphicsUpdateInterface {
    public:
      ItemManager(ControlCenter *c); 
      virtual ~ItemManager(){}
      virtual int test();

      virtual void updateItemDynamics(interfaces::sReal calc_ms, bool physics_thread = true);
      
      //virtual void setPosition(interfaces::NodeId id, const utils::Vector &pos);
      virtual const utils::Vector getPosition(interfaces::NodeId id) const;
      //virtual void setRotation(interfaces::NodeId id, const utils::Quaternion &rot);
      virtual const utils::Quaternion getRotation(interfaces::NodeId id) const;  
      virtual unsigned long getMaxGroupID() { return maxGroupID; }   
      virtual void reloadNodes(bool reloadGraphics);         
      virtual void preGraphicsUpdate(void);         
       

    private:
      interfaces::ControlCenter *control;
      
      NodeMap simNodes;          
      NodeMap simNodesDyn; 
      mutable utils::Mutex iMutex;   
      int visual_rep;

      ItemNodeData itemNodeData;

      interfaces::NodeId next_node_id;
      bool update_all_nodes;

      NodeMap nodesToUpdate;
      std::list<interfaces::NodeData> simNodesReload;
      //unsigned long maxGroupID;

      std::list<interfaces::NodeData>::iterator getReloadNode(interfaces::NodeId id);

      // interfaces::NodeInterface* getNodeInterface(NodeId node_id);
      struct Params; // see below.
      // recursively walks through the gids and joints and
      // applies the applyFunc with the given parameters.
      void recursiveHelper(interfaces::NodeId id, const Params *params,
                           std::vector<SimJoint*> *joints,
                           std::vector<int> *gids,
                           NodeMap *nodes,
                           void (*applyFunc)(SimNode *node, const Params *params));
      void moveNodeRecursive(interfaces::NodeId id, const utils::Vector &offset,
                             std::vector<SimJoint*> *joints,
                             std::vector<int> *gids,
                             NodeMap *nodes);
      void rotateNodeRecursive(interfaces::NodeId id,
                               const utils::Vector &rotation_point,
                               const utils::Quaternion &rotation,
                               std::vector<SimJoint*> *joints,
                               std::vector<int> *gids,
                               NodeMap *nodes);
      // these static methods are used by moveNodeRecursive and rotateNodeRecursive
      // as applyFuncs for the recursiveHelper method
      static void applyMove(SimNode *node, const Params *params);
      static void applyRotation(SimNode *node, const Params *params);

      void moveRelativeNodes(const SimNode &node, NodeMap *nodes, utils::Vector v);
      void rotateRelativeNodes(const SimNode &node, NodeMap *nodes,
                               utils::Vector pivot, utils::Quaternion rot);

      void resetRelativeNodes(const SimNode &node,
                              NodeMap *nodes,
                              const utils::Quaternion *rotate = 0);
      void resetRelativeJoints(const SimNode &node,
                               NodeMap *nodes,
                               std::vector<SimJoint*> *joints,
                               const utils::Quaternion *rotate = 0);
      void setNodeStructPositionFromRelative(interfaces::NodeData *node) const;
      void clearRelativePosition(interfaces::NodeId id, bool lock);
      void removeNode(interfaces::NodeId id, bool lock,
                      bool clearGraphics=true);
      void pushToUpdate(SimNode* node);

      // for passing parameters to the recursiveHelper.
      struct Params
      {
        // make virtual so we can use polymorphism
        virtual ~Params() {}
      };
      struct MoveParams : Params
      {
        utils::Vector offset;
      };
      struct RotationParams : Params
      {
        utils::Vector rotation_point;
        utils::Quaternion rotation;
      };
      

    };

  } // end of namespace sim
} // end of namespace mars

#endif  // ITEM_MANAGER_H
