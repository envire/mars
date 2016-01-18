#pragma once
// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <string>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>
#include <envire_core/events/ItemAddedEvent.hpp>
#include <envire_core/items/Frame.hpp>
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/TransformGraphTypes.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <mars/sim/ConfigMapItem.h>
#include <smurf/Robot.hpp>
#include <mars/interfaces/sim/NodeInterface.h>
#include <mars/interfaces/JointData.h>
#include <smurf/Inertial.hpp>
#include <urdf_model/model.h>
#include <smurf/Collidable.hpp>



namespace mars {
  
  namespace interfaces {
    class NodeInterface;
    class JointInterface;
  }
  
  namespace plugins {
    namespace envire_physics {

      /** This plugin adds all PhysicsConfigMapItems and JointConfigMapItems
       *  to the simulation and updates the transforms when the items move.
       * */
      class GraphPhysics : public mars::interfaces::MarsPluginTemplate,
                           public envire::core::GraphEventDispatcher,
                           public envire::core::GraphItemEventDispatcher<mars::sim::PhysicsConfigMapItem::Ptr>,
                           public envire::core::GraphItemEventDispatcher<mars::sim::JointConfigMapItem::Ptr>,
                           public envire::core::GraphItemEventDispatcher<envire::core::Item<smurf::Frame>::Ptr>,
                           public envire::core::GraphItemEventDispatcher<envire::core::Item<smurf::Collidable>::Ptr>,
                           public envire::core::GraphItemEventDispatcher<envire::core::Item<urdf::Collision>::Ptr>,
                           public envire::core::GraphItemEventDispatcher<envire::core::Item<smurf::Inertial>::Ptr>
      {
      public:
        GraphPhysics(lib_manager::LibManager *theManager);
        
        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("GraphPhysics"); }
        CREATE_MODULE_INFO();
        
        void init();
        void reset();
        
        void frameAdded(const envire::core::FrameAddedEvent& e);
        void frameRemoved(const envire::core::FrameRemovedEvent& e);
        void transformRemoved(const envire::core::TransformRemovedEvent& e);
        void transformAdded(const envire::core::TransformAddedEvent& e);
        void transformModified(const envire::core::TransformModifiedEvent& e);
        // Frames (links)
        

        void setPos(const envire::core::FrameId& frame, mars::interfaces::NodeData& node);
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Frame>::Ptr>& e);
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Collidable>::Ptr>& e);
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Inertial>::Ptr>& e);
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<urdf::Collision>::Ptr>& e);
        void itemAdded(const envire::core::TypedItemAddedEvent<mars::sim::PhysicsConfigMapItem::Ptr>& e);

        void update(mars::interfaces::sReal time_ms);
        /*
        template <class physicsType> void updatePositions(const envire::core::vertex_descriptor origin,
                                                          const envire::core::vertex_descriptor target,
                                                          const base::TransformWithCovariance& originToRoot);
        template <class physicsType> void updateChildPositions(const envire::core::vertex_descriptor vertex,
                                                               const base::TransformWithCovariance& frameToRoot);
                                                               */
        
        void updatePositions(const envire::core::vertex_descriptor origin,
                             const envire::core::vertex_descriptor target,
                             const base::TransformWithCovariance& originToRoot);
        void updateChildPositions(const envire::core::vertex_descriptor vertex,
                                  const base::TransformWithCovariance& frameToRoot);
        
        void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);
        
        
      private:
        void updateTree();
        bool instantiateNode(mars::interfaces::NodeData node, const envire::core::FrameId& frame);
        mars::interfaces::NodeData getCollidableNode(const smurf::Collidable& collidable, const envire::core::FrameId& frame);
        mars::interfaces::NodeData getInertialNode(const smurf::Inertial& inertial,const envire::core::FrameId& frame);
        
        envire::core::FrameId originId;
        envire::core::TreeView treeView;
        
        const bool debug = false;
        
      };
      
    }
    
  }
  
}
