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
 * \file WorldPhysics.h
 * \author Malte Roemmermann
 * \brief "WorldPhysics" includes the methods to handle the physically world.
 *
 */

#ifndef WORLD_PHYSICS_H
#define WORLD_PHYSICS_H

#ifdef _PRINT_HEADER_
  #warning "WorldPhysics.h"
#endif

//#define _VERIFY_WORLD_
//#define _DEBUG_MASS_

#include <mars/utils/Mutex.h>
#include <mars/utils/Vector.h>
#include <mars/interfaces/sim_common.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/PhysicsInterface.h>
#include <mars/interfaces/graphics/draw_structs.h>

#include <smurf/Collidable.hpp>

#include <vector>

#include <ode/ode.h>

#include <envire_core/items/Frame.hpp>
#include <envire_fcl/Collision.hpp>
#include "defines.hpp"


namespace mars {
  namespace sim {

    class NodePhysics;

    /**
     * The struct is used to handle some sensors in the physical
     * implementation with ode. The sensors are not implemented yet.
     */
    struct robot_geom {
      int type;
      int pressure1;
      int pressure2;
      dReal erp;
      dReal cfm;
      char *texture;
      dReal i_length;
      dBodyID body;
      dReal torque, force;
    };

    /**
     * The struct is used to handle a list of bodies. This is nessesary
     * to provide composite objects.
     */
    struct body_nbr_tupel {
      dBodyID body;
      int comp_group;
      unsigned int connected_geoms;
      std::vector<NodePhysics*> comp_nodes;
    };

    /**
     * Declaration of the physical class, that implements the
     * physics interface.
     */
    class WorldPhysics : public interfaces::PhysicsInterface, interfaces::DrawInterface {
    public:
      WorldPhysics(interfaces::ControlCenter *control);
      virtual ~WorldPhysics(void);
      virtual void initTheWorld(void);
      virtual void freeTheWorld(void);
      virtual void stepTheWorld(void);
      virtual bool existsWorld(void) const;
      virtual const utils::Vector getCenterOfMass(const std::vector<interfaces::NodeInterface*> &nodes)const;
      virtual void update(std::vector<interfaces::draw_item> *drawItems);
      virtual int checkCollisions(void);
      virtual interfaces::sReal getVectorCollision(const utils::Vector &pos, const utils::Vector &ray) const;

      // this functions are used by the other physical classes
      dWorldID getWorld(void) const;
      dSpaceID getSpace(void) const;
      bool getCompositeBody(int comp_group, dBodyID *body, NodePhysics *node);
      void destroyBody(dBodyID theBody, NodePhysics *node);
      dReal getWorldStep(void);
      void resetCompositeMass(dBodyID theBody);
      void moveCompositeMassCenter(dBodyID theBody, dReal x, dReal y, dReal z);
      int handleCollision(dGeomID theGeom);
      interfaces::sReal getCollisionDepth(dGeomID theGeom);
      mutable utils::Mutex iMutex;

      static interfaces::PhysicsError error;
		int testItem;
		
    private:

      utils::Mutex drawLock;
      dSpaceID space;
      dWorldID world;
      dGeomID plane;
      dJointGroupID contactgroup;
      bool world_init;
      interfaces::ControlCenter *control;
      utils::Vector old_gravity;
      interfaces::sReal old_cfm, old_erp;

      std::vector<body_nbr_tupel> comp_body_list;
      std::vector<interfaces::draw_item> draw_intern;
      std::vector<interfaces::draw_item> draw_extern;
      std::vector<dJointFeedback*> contact_feedback_list;
      bool create_contacts, log_contacts;
      int num_contacts;
      int ray_collision;
      // this functions are for the collision implementation
      void nearCallback (dGeomID o1, dGeomID o2);
      static void callbackForward(void *data, dGeomID o1, dGeomID o2);
      bool mlsInEnvire(void);
      void stepTheWorldChecks(void);
      void clearPreviousStep(void);
      std::vector<envire::core::FrameId> getAllColFrames(void);
      void computeMLSCollisions(void);
      void createContacts(const fcl::CollisionResultf & result, smurf::Collidable collidable, const envire::core::FrameId frameId);
      void createFeedbackJoints(const envire::core::FrameId frameId, const smurf::ContactParams contactParams, dContact *contactPtr, int numContacts);
      void initContactParams(dContact *contactPtr, const smurf::ContactParams contactParams, int numContacts);
      void dumpFCLResult(const fcl::CollisionResultf &result, dContact *contactPtr);
      void execStep(void);

    };

  } // end of namespace sim
} // end of namespace mars

#endif  // WORLD_PHYSICS_H
