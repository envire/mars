#pragma once
#include <envire_core/items/Item.hpp>
#include <configmaps/ConfigData.h>
#include <string>

namespace mars { namespace sim
{
  /**An item for the envire graph containing a ConfigMap */
 // class ConfigMapItem : public envire::core::Item<configmap::ConfigMap> {};
  using ConfigMapItem = envire::core::Item<configmaps::ConfigMap>;
  
  /**A ConfigMapItem that carries physics information */
  struct PhysicsConfigMapItem : public ConfigMapItem
  {
    using Ptr = ConfigMapItem::PtrType<PhysicsConfigMapItem>;
  };
  
  /**A ConfigMapItem that  carries joint information */
  struct JointConfigMapItem : public ConfigMapItem 
  {
    using Ptr = ConfigMapItem::PtrType<JointConfigMapItem>;
  };
  
}}