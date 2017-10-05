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
 * \file test.h
 * \author Arne Böckmann
 * \brief Plugin
 */

#pragma once
// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <mars/interfaces/NodeData.h>
#include <string>
#include <memory>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <smurf/Robot.hpp>
#include <smurf/Visual.hpp>

#include <envire_core/items/Item.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>
#include <envire_core/items/ItemBase.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/graph/Path.hpp>

#include <mars/sim/SimNode.h>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <maps/grid/MLSMap.hpp>

namespace envire {namespace core {
  class Transform;
}}

namespace mars {
  namespace plugins {
    namespace graph_viz_plugin {
      //TODO do we need inheritance from MLSMapsVisualization
      /**
       * A very simple plugin that tries to convert all ConfigMaps found in the
       * transform graph into NodeData and draw it.
       * */
      class EnvireGraphViz : public mars::interfaces::MarsPluginTemplate,
                             public envire::core::GraphEventDispatcher,
                             public envire::core::GraphItemEventDispatcher<envire::core::Item<smurf::Visual>>,
                             public envire::core::GraphItemEventDispatcher<envire::core::Item<smurf::Frame>>,
                             public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Collidable>>,
                             public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Joint>>,
                             public envire::core::GraphItemEventDispatcher<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>,
                             public envire::core::GraphItemEventDispatcher<envire::core::Item<maps::grid::MLSMapKalman>>,
                             public envire::core::GraphItemEventDispatcher<envire::core::Item<maps::grid::MLSMapPrecalculated>>,
                             public vizkit3d::MLSMapVisualization
      {

      public:
        EnvireGraphViz(lib_manager::LibManager *theManager);

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("EnvireGraphViz"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);
        
        
        virtual void itemAdded(const envire::core::ItemAddedEvent& e);
        virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Visual>>& e);
        virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Frame>>& e);
        virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Collidable>>& e);
        virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Joint>>& e);
        virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<maps::grid::MLSMapKalman>>& e);
        virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<maps::grid::MLSMapPrecalculated>>& e);
        virtual void frameAdded(const envire::core::FrameAddedEvent& e);

        virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>& e);

        // CFGClient methods
        virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);
      private:
        
        /**Add a visual node to the simulation */
        void addVisual(const smurf::Visual& visual, const envire::core::FrameId& frameId,
                       const boost::uuids::uuid& uuid);

        /**Adds a mesh visual to the simulation. Only call this method if you are sure that
         * visual.geometry is a MESH*/
        void addMesh(const smurf::Visual& visual, const envire::core::FrameId& frameId,
                     const boost::uuids::uuid& uuid);
        
        /**Adds a box visual to the simulation. Only call this method if you are sure that
         * visual.geometry is a BOX*/
        void addBox(const smurf::Visual& visual, const envire::core::FrameId& frameId,
                     const boost::uuids::uuid& uuid);
        
        /**Adds a sphere visual to the simulation. Only call this method if you are sure that
         * visual.geometry is a SPHERE*/
        void addSphere(const smurf::Visual& visual, const envire::core::FrameId& frameId,
                      const boost::uuids::uuid& uuid);
        
        /**Adds a cylinder visual to the simulation. Only call this method if you are sure that
         * visual.geometry is a CYLINDER*/
        void addCylinder(const smurf::Visual& visual, const envire::core::FrameId& frameId,
                         const boost::uuids::uuid& uuid);

        void setNodeDataMaterial(mars::interfaces::NodeData& nodeData, urdf::MaterialSharedPtr material) const;

        /** Set @p origin as the new origin frame.
          * This will update the tree and recalculate all draw positions.
          * The new origin item will apear at (0, 0, 0) with identity orientation.
         */
        void changeOrigin(const envire::core::FrameId& origin);
        
        /**Recalculates the tree and updates the draw positions of all items
         * @param origin The name of the current origin frame.
         */
        void updateTree(const envire::core::FrameId& origin);
        
        //update position of all visuals
        void updateVisuals();

        /**Updates the drawing position of @p vertex */              
        template <class physicsType> void updatePosition(const envire::core::GraphTraits::vertex_descriptor vertex);
        void setPos(const envire::core::FrameId& frame, mars::interfaces::NodeData& node);
	
      private:
        /**Maps the item's uuid to the graphics id used for drawing */
        std::unordered_map<boost::uuids::uuid, int, boost::hash<boost::uuids::uuid>> uuidToGraphicsId;
        std::unordered_map<boost::uuids::uuid, int, boost::hash<boost::uuids::uuid>> uuidToGraphicsId2;
        envire::core::FrameId originId; /**<id of the current origin */
        envire::core::TreeView tree; /**<tree containing all visualized vertices */
        
        //buffers the paths from origin to the nodes to avoid tree searches.
        //FIXME paths might become invalid if the graph changes.
        std::unordered_map<envire::core::GraphTraits::vertex_descriptor,std::shared_ptr<envire::core::Path>> pathsFromOrigin;
        
        bool viewCollidables = false;
        bool viewJoints = false;
        bool viewFrames = false;
        const int visualUpdateRateFps = 30;
        float timeSinceLastUpdateMs = 0; 
        osg::ref_ptr<osg::Group> osgGroup;
        osg::ref_ptr<osg::Node>  osgNode;
      }; // end of class definition TestTreeMars

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars
