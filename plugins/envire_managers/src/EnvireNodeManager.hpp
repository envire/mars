/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
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
 * \file EnvireMotors.h
 * \author Raul (raul.dominguez@dfki.de)
 * \brief Create and store simulated motors from their definition in the envire representation
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_ENVIRE_NODE_MANAGER_H
#define MARS_PLUGINS_ENVIRE_NODE_MANAGER_H

#ifdef _PRINT_HEADER_
  #warning "EnvireNodeManager.hpp"
#endif

#include <mars/utils/Mutex.h>
#include <mars/interfaces/graphics/GraphicsUpdateInterface.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>

namespace mars { 
  namespace sim {
    class SimJoint; 
    class SimNode; 
  }
}

namespace mars {
  namespace plugins {
        namespace envire_managers {

          using SimNodeItem =  envire::core::Item<std::shared_ptr<mars::sim::SimNode>>;
          using SimJointItem =  envire::core::Item<std::shared_ptr<mars::sim::SimJoint>>;
          using SimNodeItemPtr = SimNodeItem::Ptr;          

          typedef std::map<mars::interfaces::NodeId, SimNodeItemPtr> NodeMap;

          /**
           * The declaration of the EnvireNodeManager class.
           *
           */
          class EnvireNodeManager : public interfaces::NodeManagerInterface,
                              public interfaces::GraphicsUpdateInterface {
          public:
            EnvireNodeManager(mars::interfaces::ControlCenter *c,
                        lib_manager::LibManager *theManager);
            virtual ~EnvireNodeManager(){}

            virtual mars::interfaces::NodeId createPrimitiveNode(const std::string &name,
                                                           mars::interfaces::NodeType type,
                                                           bool movable=false,
                                                           const mars::utils::Vector &pos=mars::utils::Vector::Zero(),
                                                           const mars::utils::Vector &extension=mars::utils::Vector::Identity(),
                                                           double mass=0,
                                                           const mars::utils::Quaternion &orientation=mars::utils::Quaternion::Identity(),
                                                           bool disablePhysics=false);

            virtual mars::interfaces::NodeId addNode(mars::interfaces::NodeData *nodeS,
                                               bool reload = false,
                                               bool loadGraphics = true);

            virtual mars::interfaces::NodeId addTerrain(mars::interfaces::terrainStruct *terrainS);
            virtual std::vector<mars::interfaces::NodeId> addNode(std::vector<mars::interfaces::NodeData> v_NodeData);
            virtual mars::interfaces::NodeId addPrimitive(mars::interfaces::NodeData *snode);
            virtual int getNodeCount() const;
            virtual mars::interfaces::NodeId getNextNodeID() const;
            virtual void editNode(mars::interfaces::NodeData *nodeS, int changes);
            virtual void changeGroup(mars::interfaces::NodeId id, int group);
            virtual void getListNodes(std::vector<mars::interfaces::core_objects_exchange> *nodeList) const;
            virtual void getNodeExchange(mars::interfaces::NodeId id,
                                         mars::interfaces::core_objects_exchange *obj) const;
            virtual const mars::interfaces::NodeData getFullNode(mars::interfaces::NodeId id) const;
            virtual void removeNode(mars::interfaces::NodeId id, bool clearGraphics=true);
            virtual void setNodeState(mars::interfaces::NodeId id, const mars::interfaces::nodeState &state);
            virtual void getNodeState(mars::interfaces::NodeId id, mars::interfaces::nodeState *state) const;
            virtual const mars::utils::Vector getCenterOfMass(const std::vector<mars::interfaces::NodeId> &ids) const;
            virtual void setPosition(mars::interfaces::NodeId id, const mars::utils::Vector &pos);
            virtual const mars::utils::Vector getPosition(mars::interfaces::NodeId id) const;
            virtual void setRotation(mars::interfaces::NodeId id, const mars::utils::Quaternion &rot);
            virtual const mars::utils::Quaternion getRotation(mars::interfaces::NodeId id) const;
            virtual const mars::utils::Vector getLinearVelocity(mars::interfaces::NodeId id) const;
            virtual const mars::utils::Vector getAngularVelocity(mars::interfaces::NodeId id) const;
            virtual const mars::utils::Vector getLinearAcceleration(mars::interfaces::NodeId id) const;
            virtual const mars::utils::Vector getAngularAcceleration(mars::interfaces::NodeId id) const;
            virtual void applyForce(mars::interfaces::NodeId id, const mars::utils::Vector &force,
                                    const mars::utils::Vector &pos);
            virtual void applyForce(mars::interfaces::NodeId id, const mars::utils::Vector &force);
            virtual void applyTorque(mars::interfaces::NodeId id, const mars::utils::Vector &torque);
            virtual void setContactParamMotion1(mars::interfaces::NodeId id, mars::interfaces::sReal motion);
            virtual void addNodeSensor(mars::interfaces::BaseNodeSensor *sensor);
            virtual void reloadNodeSensor(mars::interfaces::BaseNodeSensor *sensor);
            virtual std::shared_ptr<mars::sim::SimNode> getSimNode(mars::interfaces::NodeId id);
            virtual void reloadNodes(bool reloadGraphics);
            virtual const mars::utils::Vector setReloadExtent(mars::interfaces::NodeId id, const mars::utils::Vector &ext);
            virtual void setReloadPosition(mars::interfaces::NodeId id, const mars::utils::Vector &pos);
            virtual void setReloadFriction(mars::interfaces::NodeId id, mars::interfaces::sReal friction1,
                                           mars::interfaces::sReal friction2);
            virtual void updateDynamicNodes(mars::interfaces::sReal calc_ms, bool physics_thread = true);
            virtual void clearAllNodes(bool clear_all=false, bool clearGraphics=true);
            virtual void setReloadAngle(mars::interfaces::NodeId id, const mars::utils::sRotation &angle);
            virtual void setContactParams(mars::interfaces::NodeId id, const mars::interfaces::contact_params &cp);
            virtual const mars::interfaces::contact_params getContactParams(mars::interfaces::NodeId id) const;
            virtual void setVelocity(mars::interfaces::NodeId id, const mars::utils::Vector& vel);
            virtual void setAngularVelocity(mars::interfaces::NodeId id, const mars::utils::Vector &vel);
            virtual void scaleReloadNodes(mars::interfaces::sReal x, mars::interfaces::sReal y, mars::interfaces::sReal z);
            virtual void getNodeMass(mars::interfaces::NodeId id, mars::interfaces::sReal *mass, mars::interfaces::sReal *inertia = 0) const;
            virtual void setAngularDamping(mars::interfaces::NodeId id, mars::interfaces::sReal damping);
            virtual void addRotation(mars::interfaces::NodeId id, const mars::utils::Quaternion &q);
            virtual void setReloadQuaternion(mars::interfaces::NodeId id, const mars::utils::Quaternion &q);
            virtual void preGraphicsUpdate(void);
            virtual void exportGraphicNodesByID(const std::string &folder) const;
            virtual void getContactPoints(std::vector<mars::interfaces::NodeId> *ids,
                                          std::vector<mars::utils::Vector> *contact_points) const;
            virtual void getContactIDs(const mars::interfaces::NodeId &id,
                                       std::list<mars::interfaces::NodeId> *ids) const;
            virtual void updateRay(mars::interfaces::NodeId id);
            virtual mars::interfaces::NodeId getDrawID(mars::interfaces::NodeId id) const;
            virtual void setVisualRep(mars::interfaces::NodeId id, int val);
            virtual const mars::utils::Vector getContactForce(mars::interfaces::NodeId id) const;
            virtual void setVisualQOffset(mars::interfaces::NodeId id, const mars::utils::Quaternion &q);

            virtual void updatePR(mars::interfaces::NodeId id, const mars::utils::Vector &pos,
                                  const mars::utils::Quaternion &rot,
                                  const mars::utils::Vector &visOffsetPos,
                                  const mars::utils::Quaternion &visOffsetRot,
                                  bool doLock = true);
            /**
             * Retrieve the id of a node by name
             * \param node_name Name of the node to get the id for
             * \return Id of the node if it exists, otherwise 0
             */
            virtual mars::interfaces::NodeId getID(const std::string& node_name) const;
            virtual double getCollisionDepth(mars::interfaces::NodeId id) const;
            virtual bool getDataBrokerNames(mars::interfaces::NodeId id, std::string *groupName,
                                            std::string *dataName) const;

            virtual std::vector<mars::interfaces::NodeId> getConnectedNodes(mars::interfaces::NodeId id);

            virtual bool getIsMovable(mars::interfaces::NodeId id) const;
            virtual void setIsMovable(mars::interfaces::NodeId id, bool isMovable);
            virtual void lock() {iMutex.lock();}
            virtual void unlock() {iMutex.unlock();}
            virtual void rotateNode(mars::interfaces::NodeId id, mars::utils::Vector pivot,
                                    mars::utils::Quaternion q,
                                    unsigned long excludeJointId, bool includeConnected = true);
            virtual void positionNode(mars::interfaces::NodeId id, mars::utils::Vector pos,
                                      unsigned long excludeJointId);
            virtual unsigned long getMaxGroupID() { return maxGroupID; }
            virtual void edit(mars::interfaces::NodeId id, const std::string &key,
                              const std::string &value);

            virtual void updatePositionsFromGraph();

          private:
            mars::interfaces::NodeId next_node_id;
            bool update_all_nodes;
            int visual_rep;
            NodeMap simNodes;
            //NodeMap simNodesDyn;
            NodeMap nodesToUpdate;
            //std::list<mars::interfaces::NodeData> simNodesReload;
            int maxGroupID;
            lib_manager::LibManager *libManager;
            mutable mars::utils::Mutex iMutex;

            mars::interfaces::ControlCenter *control;

            envire::core::TreeView graphTreeView;

            std::list<mars::interfaces::NodeData>::iterator getReloadNode(mars::interfaces::NodeId id);

            // interfaces::NodeInterface* getNodeInterface(NodeId node_id);
            struct Params; // see below.
            // recursively walks through the gids and joints and
            // applies the applyFunc with the given parameters.
            void recursiveHelper(mars::interfaces::NodeId id, const Params *params,
                                 std::vector<mars::sim::SimJoint*> *joints,
                                 std::vector<int> *gids,
                                 NodeMap *nodes,
                                 void (*applyFunc)(mars::sim::SimNode *node, const Params *params));
            void moveNodeRecursive(mars::interfaces::NodeId id, const mars::utils::Vector &offset,
                                   std::vector<mars::sim::SimJoint*> *joints,
                                   std::vector<int> *gids,
                                   NodeMap *nodes);
            void rotateNodeRecursive(mars::interfaces::NodeId id,
                                     const mars::utils::Vector &rotation_point,
                                     const mars::utils::Quaternion &rotation,
                                     std::vector<mars::sim::SimJoint*> *joints,
                                     std::vector<int> *gids,
                                     NodeMap *nodes);
            // these static methods are used by moveNodeRecursive and rotateNodeRecursive
            // as applyFuncs for the recursiveHelper method
            static void applyMove(mars::sim::SimNode *node, const Params *params);
            static void applyRotation(mars::sim::SimNode *node, const Params *params);

            void moveRelativeNodes(const mars::sim::SimNode &node, NodeMap *nodes, mars::utils::Vector v);
            void rotateRelativeNodes(const mars::sim::SimNode &node, NodeMap *nodes,
                                     mars::utils::Vector pivot, mars::utils::Quaternion rot);

            void resetRelativeNodes(const mars::sim::SimNode &node,
                                    NodeMap *nodes,
                                    const mars::utils::Quaternion *rotate = 0);
            void resetRelativeJoints(const mars::sim::SimNode &node,
                                     NodeMap *nodes,
                                     std::vector<mars::sim::SimJoint*> *joints,
                                     const mars::utils::Quaternion *rotate = 0);
            void setNodeStructPositionFromRelative(mars::interfaces::NodeData *node) const;
            void clearRelativePosition(mars::interfaces::NodeId id, bool lock);
            void removeNode(mars::interfaces::NodeId id, bool lock,
                            bool clearGraphics=true);
            void pushToUpdate(mars::sim::SimNode* node);

            void printNodeMasses(bool onlysum);

            void updateChildPositions(const envire::core::GraphTraits::vertex_descriptor vertex,
                                      const base::TransformWithCovariance& frameToRoot,
                                      mars::interfaces::sReal calc_ms, bool physics_thread);



            void updatePositions(const envire::core::GraphTraits::vertex_descriptor origin,
                                 const envire::core::GraphTraits::vertex_descriptor target,
                                 const base::TransformWithCovariance& originToRoot,
                                 mars::interfaces::sReal calc_ms, bool physics_thread);            

            // for passing parameters to the recursiveHelper.
            struct Params
            {
              // make virtual so we can use polymorphism
              virtual ~Params() {}
            };
            struct MoveParams : Params
            {
              mars::utils::Vector offset;
            };
            struct RotationParams : Params
            {
              mars::utils::Vector rotation_point;
              mars::utils::Quaternion rotation;
            };

          };

    }
  } // end of namespace sim
} // end of namespace mars

#endif  // NODE_MANAGER_H
