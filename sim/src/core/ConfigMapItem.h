#pragma once
#include <envire_core/items/Item.hpp>
#include <configmaps/ConfigData.h>

namespace mars { namespace sim
{
  /**An item for the envire graph containing a ConfigMap */
 // class ConfigMapItem : public envire::core::Item<configmap::ConfigMap> {};
  using ConfigMapItem = envire::core::Item<configmaps::ConfigMap>;
  
  /**A ConfigMapItem that carries physics information */
  struct PhysicsConfigMapItem : public ConfigMapItem {};
}}