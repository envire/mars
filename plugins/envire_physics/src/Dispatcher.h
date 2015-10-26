#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/graph/TransformGraph.hpp>
#include <envire_core/events/TransformAddedEvent.hpp>
#include <envire_core/events/TransformModifiedEvent.hpp>
#include <envire_core/events/TransformRemovedEvent.hpp>
#include <envire_core/events/FrameAddedEvent.hpp>

namespace mars{
    namespace envire_physics{

      class Dispatcher : public envire::core::GraphEventDispatcher {
        public:
          std::vector<envire::core::TransformAddedEvent> transformAddedEvents;
          std::vector<envire::core::TransformModifiedEvent> transformModifiedEvents;
          std::vector<envire::core::TransformRemovedEvent> transformRemovedEvents;
          std::vector<envire::core::FrameAddedEvent> frameAddedEvents;

          Dispatcher(envire::core::TransformGraph& graph) : envire::core::GraphEventDispatcher(graph) {}
          virtual ~Dispatcher() {}

          virtual void transformAdded(const envire::core::TransformAddedEvent& e)
          {
              transformAddedEvents.push_back(e);
          }

          virtual void transformModified(const envire::core::TransformModifiedEvent& e)
          {
              transformModifiedEvents.push_back(e);
          }
	  
          virtual void transformRemoved(const envire::core::TransformRemovedEvent& e)
          {
              transformRemovedEvents.push_back(e);
          }
          
          
          virtual void frameAdded(const envire::core::FrameAddedEvent& e)
          {
              frameAddedEvents.push_back(e);
          }
      };
    }
}
