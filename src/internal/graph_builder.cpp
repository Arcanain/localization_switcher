// src/graph_builder.cpp
#include "localization_switcher/internal/graph_builder.hpp"
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace localization_switcher
{
  namespace
  {
    // 大文字化（状態語の正規化）
    std::string to_upper(std::string s)
    {
      std::transform(s.begin(), s.end(), s.begin(),
                      [](unsigned char c)
                      { return static_cast<char>(std::toupper(c)); });
      return s;
    }

    // 文字列 → SemanticState::State（大小無視）
    bool parse_state_string(const std::string &state_str, SemanticState::State &out)
    {
      const std::string u = to_upper(state_str);
      if (u == "UNKNOWN")
      {
        out = SemanticState::State::UNKNOWN;
        return true;
      }
      if (u == "UNCONFIGURED")
      {
        out = SemanticState::State::UNCONFIGURED;
        return true;
      }
      if (u == "INACTIVE")
      {
        out = SemanticState::State::INACTIVE;
        return true;
      }
      if (u == "ACTIVE")
      {
        out = SemanticState::State::ACTIVE;
        return true;
      }
      if (u == "FINALIZED")
      {
        out = SemanticState::State::FINALIZED;
        return true;
      }
      return false;
    }

    // スカラーdoubleの取り出し（既定値つき）
    double get_scalar_double_or(const YAML::Node &node, double fallback)
    {
      return (node && node.IsScalar()) ? node.as<double>() : fallback;
    }

    // スカラーboolの取り出し（既定値つき）
    bool get_scalar_bool_or(const YAML::Node &node, bool fallback)
    {
      return (node && node.IsScalar()) ? node.as<bool>() : fallback;
    }

    // nodes を一旦蓄える中間表現
    struct TempNode
    {
      std::string id;
      SemanticState semantic; // 実ノード名 -> 状態
      bool allowed{true};
      std::unordered_map<std::string, TransitionRecipe> outgoing; // to_id -> recipe
    };
  } // namespace

  std::optional<Graph> GraphBuilder::build_from_yaml(const std::string &yaml_path)
  {
    auto fail = [](const std::string &message) -> std::optional<Graph>
    {
      std::cerr << "[GraphBuilder] " << message << std::endl;
      return std::nullopt;
    };

    YAML::Node root_doc;
    try
    {
      root_doc = YAML::LoadFile(yaml_path);
    }
    catch (const std::exception &e)
    {
      return fail(std::string("YAML load failed: ") + e.what());
    }
    if (!root_doc || !root_doc.IsMap())
      return fail("Top-level YAML node must be a mapping.");

    // ---- lifecyclenodes: 実ノード名（文字列）の列 ----
    const YAML::Node lifecyclenodes_seq = root_doc["lifecyclenodes"];
    if (!lifecyclenodes_seq || !lifecyclenodes_seq.IsSequence())
      return fail("'lifecyclenodes' must be a sequence of node names.");

    std::unordered_set<std::string> lifecycle_node_names;
    lifecycle_node_names.reserve(lifecyclenodes_seq.size());

    for (std::size_t node_i = 0; node_i < lifecyclenodes_seq.size(); ++node_i)
    {
      const YAML::Node name_node = lifecyclenodes_seq[node_i];
      if (!name_node.IsScalar())
        return fail("lifecyclenodes[" + std::to_string(node_i) + "] must be a scalar string.");
      const std::string node_name = name_node.as<std::string>();
      if (!lifecycle_node_names.insert(node_name).second)
        return fail("Duplicate node name in lifecyclenodes: '" + node_name + "'.");
    }
    if (lifecycle_node_names.empty())
      return fail("lifecyclenodes must not be empty.");

    // ---- nodes ----
    const YAML::Node nodes_seq = root_doc["nodes"];
    if (!nodes_seq || !nodes_seq.IsSequence())
      return fail("'nodes' must be a sequence.");

    std::unordered_map<std::string, TempNode> accumulators_by_id;
    accumulators_by_id.reserve(nodes_seq.size());

    for (std::size_t idx = 0; idx < nodes_seq.size(); ++idx)
    {
      const YAML::Node node_map = nodes_seq[idx];
      if (!node_map.IsMap()) return fail("nodes[" + std::to_string(idx) + "] must be a mapping.");

      const YAML::Node id_node = node_map["id"];
      if (!id_node || !id_node.IsScalar()) return fail("nodes[" + std::to_string(idx) + "].id must be scalar.");
      const std::string state_id = id_node.as<std::string>();
      if (accumulators_by_id.count(state_id))
        return fail("Duplicate node id: '" + state_id + "'.");

      TempNode temp_node;
      temp_node.id = state_id;
      temp_node.allowed = get_scalar_bool_or(node_map["allowed"], true);

      const YAML::Node semantic_map = node_map["semantic"];
      if (!semantic_map || !semantic_map.IsMap())
        return fail("nodes[" + state_id + "].semantic must be a mapping of {node_name: STATE}.");

      for (auto it = semantic_map.begin(); it != semantic_map.end(); ++it)
      {
        const std::string node_name = it->first.as<std::string>(); // 実ノード名
        if (!lifecycle_node_names.count(node_name))
          return fail("nodes[" + state_id + "].semantic has unknown node '" + node_name + "'.");
        if (!it->second.IsScalar())
          return fail("nodes[" + state_id + "].semantic['" + node_name + "'] must be scalar.");

        SemanticState::State parsed;
        if (!parse_state_string(it->second.as<std::string>(), parsed))
          return fail("nodes[" + state_id + "].semantic['" + node_name + "'] has invalid state.");

        temp_node.semantic.semantic_state[node_name] = parsed;
      }

      if (temp_node.semantic.semantic_state.size() != lifecycle_node_names.size())
        return fail("nodes[" + state_id + "].semantic must specify all lifecyclenodes.");

      accumulators_by_id.emplace(state_id, std::move(temp_node));
    }

    // ---- edges ----
    const YAML::Node edges_seq = root_doc["edges"];
    if (!edges_seq || !edges_seq.IsSequence())
      return fail("'edges' must be a sequence.");

    for (std::size_t edge_i = 0; edge_i < edges_seq.size(); ++edge_i)
    {
      const YAML::Node edge_map = edges_seq[edge_i];
      if (!edge_map.IsMap())
        return fail("edges[" + std::to_string(edge_i) + "] must be a mapping.");

      const YAML::Node from_node = edge_map["from"];
      const YAML::Node to_node = edge_map["to"];
      if (!from_node || !from_node.IsScalar() || !to_node || !to_node.IsScalar())
        return fail("edges[" + std::to_string(edge_i) + "] requires scalar 'from' and 'to'.");

      const std::string from_id = from_node.as<std::string>();
      const std::string to_id = to_node.as<std::string>();

      auto from_it = accumulators_by_id.find(from_id);
      if (from_it == accumulators_by_id.end())
        return fail("edges[" + std::to_string(edge_i) + "].from '" + from_id + "' not found in nodes.");
      if (!accumulators_by_id.count(to_id))
        return fail("edges[" + std::to_string(edge_i) + "].to '" + to_id + "' not found in nodes.");

      TransitionRecipe recipe; // 空でも可
      const YAML::Node recipe_map = edge_map["transitionrecipe"];
      if (recipe_map && recipe_map.IsMap())
      {
        const YAML::Node steps_seq = recipe_map["actionsteps"];
        if (steps_seq && steps_seq.IsSequence())
        {
          recipe.steps.reserve(steps_seq.size());
          for (std::size_t step_i = 0; step_i < steps_seq.size(); ++step_i)
          {
            const YAML::Node step_map = steps_seq[step_i];
            if (!step_map.IsMap())
              return fail("edges[" + std::to_string(edge_i) + "].steps[" + std::to_string(step_i) + "] must be a mapping.");

            const YAML::Node target_node_name_node = step_map["target_node_name"];
            const YAML::Node operation_node = step_map["operation"];
            const YAML::Node timeout_node = step_map["timeout_s"];

            if (!target_node_name_node || !target_node_name_node.IsScalar() ||
                !operation_node || !operation_node.IsScalar())
              return fail("edges[" + std::to_string(edge_i) + "].steps[" + std::to_string(step_i) + "] requires 'target_node_name' and 'operation'.");

            const std::string target_node_name = target_node_name_node.as<std::string>();
            if (!lifecycle_node_names.count(target_node_name))
              return fail("edges[" + std::to_string(edge_i) + "].steps[" + std::to_string(step_i) + "]: unknown node '" + target_node_name + "'.");

            ActionStep step;
            step.target_node_name = target_node_name;
            step.operation = operation_node.as<std::string>();
            step.timeout_s = get_scalar_double_or(timeout_node, 5.0);
            recipe.steps.push_back(std::move(step));
          }
        }
      }

      TempNode &from_acc = from_it->second;
      if (from_acc.outgoing.count(to_id))
        return fail("Duplicate edge from '" + from_id + "' to '" + to_id + "'.");
      from_acc.outgoing.emplace(to_id, std::move(recipe));
    }

    // ---- Graph 構築 ----
    std::vector<Node> nodes;
    nodes.reserve(accumulators_by_id.size());
    for (auto &kv : accumulators_by_id)
    {
      TempNode &temp_node = kv.second;
      nodes.emplace_back(temp_node.id, std::move(temp_node.outgoing), std::move(temp_node.semantic));
    }
    return Graph(std::move(nodes));
  }

} // namespace localization_switcher
