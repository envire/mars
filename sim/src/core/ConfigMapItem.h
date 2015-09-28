#pragma once
#include <envire_core/Item.hpp>
#include <configmaps/ConfigData.h>

namespace mars { namespace sim
{
  /**An item for the envire graph containing a ConfigMap */
  class ConfigMapItem : public envire::core::Item<configmaps::ConfigMap> {};
}}