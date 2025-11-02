// include/localization_switcher/internal/graph_builder.hpp
#pragma once
#include "localization_switcher/lib/graph.hpp"
#include "localization_switcher/lib/node.hpp"
#include "localization_switcher/common_types.hpp"
#include <optional>
#include <string>

namespace localization_switcher
{
  class GraphBuilder
  {
  public:
    static std::optional<Graph> build_from_yaml(const std::string &yaml_path);
  };
}
