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
#include <string>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/items/Frame.hpp>
#include <envire_core/graph/TransformGraphTypes.hpp>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>

namespace envire {namespace core {
  class TransformGraph;
  class Transform;
}}

namespace mars {
  namespace plugins {
    namespace graph_viz_plugin {

      /**
       * A very simple plugin that tries to convert all ConfigMaps found in the
       * transform graph into NodeData and draw it.
       * */
      class GraphViz : public mars::interfaces::MarsPluginTemplate,
                       public envire::core::GraphEventDispatcher
      {

      public:
        GraphViz(lib_manager::LibManager *theManager);

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("GraphViz"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);
        
        
        virtual void transformAdded(const envire::core::TransformAddedEvent& e);
        virtual void transformRemoved(const envire::core::TransformRemovedEvent& e);
        virtual void transformModified(const envire::core::TransformModifiedEvent& e);
        virtual void itemAdded(const envire::core::ItemAddedEvent& e);
        virtual void frameAdded(const envire::core::FrameAddedEvent& e);

        // CFGClient methods
        virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);
      private:
        
        /** Set @p origin as the new origin frame.
          * This will update the tree and recalculate all draw positions.
          * The new origin item will apear at (0, 0, 0) with identity orientation.
         */
        void changeOrigin(const envire::core::FrameId& origin);
        
        /**Recalculates the tree and updates the draw positions of all items
         * @param origin The name of the current origin frame.
         */
        void updateTree(const envire::core::FrameId& origin);
        
        /**Determine whether @p a is the parent of @p b */
        bool isParent(const envire::core::vertex_descriptor a,
                      const envire::core::vertex_descriptor b) const;
                      
        /**Updates the drawing position of @p vertex */              
        void updatePosition(const envire::core::vertex_descriptor vertex) const;
      private:
        /**Maps the item's uuid to the graphics id used for drawing */
        std::unordered_map<boost::uuids::uuid, int, boost::hash<boost::uuids::uuid>> uuidToGraphicsId;
        envire::core::FrameId originId; /**<id of the current origin */
        envire::core::VertexRelationMap tree; /**<map from parent to children */
        

      }; // end of class definition TestTreeMars

    } // end of namespace TestTreeMars
  } // end of namespace plugins
} // end of namespace mars