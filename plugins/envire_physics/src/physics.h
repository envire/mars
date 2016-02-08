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
                           public envire::core::GraphItemEventDispatcher<envire::core::Item<configmaps::ConfigMap>>,
                           public envire::core::GraphItemEventDispatcher<mars::sim::PhysicsConfigMapItem>,
                           public envire::core::GraphItemEventDispatcher<mars::sim::JointConfigMapItem>,
                           public envire::core::GraphItemEventDispatcher<envire::core::Item<smurf::Frame>>,
                           public envire::core::GraphItemEventDispatcher<envire::core::Item<smurf::Collidable>>,
                           public envire::core::GraphItemEventDispatcher<envire::core::Item<urdf::Collision>>,
                           public envire::core::GraphItemEventDispatcher<envire::core::Item<smurf::Inertial>>
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
        /** 
         * When a smurf::Frame object is introduced a NodeData is created with
         * simple shape and that can not collide with other objects. The 
         * correspondent physical node is instantiated in the physical 
         * simulator. The interface to the simulated object is stored through a
         * shared_ptr in the frame where the Collidable was found. The nodes 
         * created by the storage of a smurf Frame are the ones the joints 
         * link.
         */
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Frame>>& e);
        /**
         * When a smurf::Collidable objects is introduced a NodeData is 
         * generated with the information of the collidable and it is 
         * instantiated in the physical simulator. The interface to the 
         * simulated object is stored through a shared_ptr in the frame where 
         * the Collidable was found.
         */
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Collidable>>& e);
        /**
         * When a smurf::Inertial objects is introduced, a NodeData is 
         * generated with the information of the inertial and it is 
         * instantiated in the physical simulator. The interface to the 
         * simulated object is stored through a shared_ptr in the frame where 
         * the Inertial was found.
         */
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<smurf::Inertial>>& e);
        /**
         * Same as for the method that detects a new collidable object, in this
         * case the simulated node will be based only on the urdf collision 
         * object.
         */
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<urdf::Collision>>& e);
        /**
         * When a PhysicsConfigMapItem objects is introduced, a NodeData is 
         * generated with the information of the object and it is 
         * instantiated in the physical simulator. The interface to the 
         * simulated object is stored through a shared_ptr in the frame where 
         * the PhysicsConfigMapItem was found.
         */
        void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<configmaps::ConfigMap>>& e);
        void itemAdded(const envire::core::TypedItemAddedEvent<mars::sim::PhysicsConfigMapItem>& e);
        /*
         *  dfs visit the tree and update all positions.
         *  The transforms in the graph are relative to their parent while the
         *  transform from simulation is relative to the root.
         *  The relative:w
         * transformations are easy to calculate when dfs visiting the tree.
         */
        void update(mars::interfaces::sReal time_ms);

        void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);
        
      private:
        void updateTree();
        /**
         * Returns a Nodedata with the configuration provided by the smurf collidable and positioned according to the frame
         */
        mars::interfaces::NodeData getCollidableNode(const smurf::Collidable& collidable, const envire::core::FrameId& frame);
        /**
         * Returns a NodeData configured with the data provided by the smurf::Inertial object and positioned according to the frame
         */
        mars::interfaces::NodeData getInertialNode(const smurf::Inertial& inertial,const envire::core::FrameId& frame);        
        /*
         * Create the physical objects and save them in the Graph
         * We add the shared_ptr of the physical node interface to access to the physical simulation of the object
         */
        bool instantiateNode(mars::interfaces::NodeData node, const envire::core::FrameId& frame);
        /*
         * Sets to the nodeData the position that corresponds to the given frame id
         */
        void setPos(const envire::core::FrameId& frame, mars::interfaces::NodeData& node);

        /*
         *  Perform updatePositions for each of your childs
         */
        void updateChildPositions(const envire::core::vertex_descriptor vertex,
                                  const base::TransformWithCovariance& frameToRoot);
        void updatePositions(const envire::core::vertex_descriptor origin,
                             const envire::core::vertex_descriptor target,
                             const base::TransformWithCovariance& originToRoot);
        
        envire::core::FrameId originId;
        envire::core::TreeView treeView;
        
        const bool debug = false;
        const bool printGraph = false;
        const bool debugUpdatePos = false;
        
      };
      
    }
    
  }
  
}
