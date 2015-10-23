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


namespace mars {
  
  namespace interfaces {
    class NodeInterface;
  }
  
  namespace plugins {
    namespace envire_physics {

      class GraphPhysics : public mars::interfaces::MarsPluginTemplate,
                   public envire::core::GraphEventDispatcher
      {
      public:
        GraphPhysics(lib_manager::LibManager *theManager);
        ~GraphPhysics();
        
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
        void itemAdded(const envire::core::ItemAddedEvent& e);
        void update(mars::interfaces::sReal time_ms);
        void updatePositions(const envire::core::vertex_descriptor origin,
                             const envire::core::vertex_descriptor target,
                             const base::TransformWithCovariance& originToRoot);
        void updateChildPositions(const envire::core::vertex_descriptor vertex,
                                  const base::TransformWithCovariance& frameToRoot);
        
        void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);
        
        
      private:
        envire::core::FrameId originId;
        envire::core::VertexMap tree;
        std::unordered_map<boost::uuids::uuid, std::shared_ptr<interfaces::NodeInterface>, boost::hash<boost::uuids::uuid>> uuidToPhysics;
      };

}}}
