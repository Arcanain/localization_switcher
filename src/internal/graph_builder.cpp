// src/graph_builder.cpp
#include "localization_switcher/internal/graph_builder.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace localization_switcher
{

  namespace
  {

    // 大文字化（状態語の正規化用）
    std::string to_upper(std::string s)
    {
      std::transform(s.begin(), s.end(), s.begin(),
                     [](unsigned char c)
                     { return static_cast<char>(std::toupper(c)); });
      return s;
    }

    // 文字列 → SemanticState::State（大小無視）
    bool parse_state(const std::string &s, SemanticState::State &out)
    {
      const std::string u = to_upper(s);
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

    // YAML ノードから double を安全に取り出す（既定値つき）
    double as_double_or(const YAML::Node &n, double fallback)
    {
      if (!n || n.IsNull())
        return fallback;
      if (n.IsScalar())
        return n.as<double>();
      return fallback;
    }

    // YAML ノードから bool を安全に取り出す（既定値つき）
    bool as_bool_or(const YAML::Node &n, bool fallback)
    {
      if (!n || n.IsNull())
        return fallback;
      if (n.IsScalar())
        return n.as<bool>();
      return fallback;
    }

    struct TmpNode
    {
      std::string id;
      SemanticState semantic;
      bool allowed{true};
      std::unordered_map<std::string, TransitionRecipe> next; // to_id -> recipe
    };

  } // anonymous namespace

  std::optional<Graph> GraphBuilder::build_from_yaml(const std::string &yaml_path)
  {
    auto fail = [](const std::string &msg) -> std::optional<Graph>
    {
      std::cerr << "GraphBuilder error: " << msg << std::endl;
      return std::nullopt;
    };

    YAML::Node root;
    try
    {
      root = YAML::LoadFile(yaml_path);
    }
    catch (const std::exception &e)
    {
      return fail(std::string("YAML load failed: ") + e.what());
    }

    if (!root || !root.IsMap())
    {
      return fail("Top-level YAML node must be a mapping.");
    }

    // lifecyclenodes: キー集合（semantic のキー検証、recipe の target_key 検証に使用）
    const YAML::Node y_lc = root["lifecyclenodes"];
    if (!y_lc || !y_lc.IsMap())
    {
      return fail("'lifecyclenodes' must exist and be a mapping.");
    }

    std::unordered_map<std::string, std::string> lifecycle_keys; // key -> node_name
    for (auto it = y_lc.begin(); it != y_lc.end(); ++it)
    {
      const std::string key = it->first.as<std::string>();
      const YAML::Node v = it->second;
      const YAML::Node name = v["node_name"];
      if (!name || !name.IsScalar())
      {
        return fail("lifecyclenodes['" + key + "'].node_name must be scalar.");
      }
      if (lifecycle_keys.count(key))
      {
        return fail("Duplicate lifecyclenodes key: '" + key + "'.");
      }
      lifecycle_keys.emplace(key, name.as<std::string>());
    }
    if (lifecycle_keys.empty())
    {
      return fail("lifecyclenodes must contain at least one entry.");
    }

    // nodes: まず中間表現に溜める（edges 取り込み後に Node を一括生成するため）
    const YAML::Node y_nodes = root["nodes"];
    if (!y_nodes || !y_nodes.IsSequence())
    {
      return fail("'nodes' must exist and be a sequence.");
    }

    std::unordered_map<std::string, TmpNode> tmp_by_id;
    tmp_by_id.reserve(y_nodes.size());

    for (std::size_t i = 0; i < y_nodes.size(); ++i)
    {
      const YAML::Node yn = y_nodes[i];
      if (!yn.IsMap())
      {
        return fail("nodes[" + std::to_string(i) + "] must be a mapping.");
      }

      const YAML::Node y_id = yn["id"];
      if (!y_id || !y_id.IsScalar())
      {
        return fail("nodes[" + std::to_string(i) + "].id must be scalar.");
      }
      const std::string id = y_id.as<std::string>();
      if (tmp_by_id.count(id))
      {
        return fail("Duplicate node id: '" + id + "'.");
      }

      TmpNode tmp;
      tmp.id = id;
      tmp.allowed = as_bool_or(yn["allowed"], true);

      const YAML::Node y_sem = yn["semantic"];
      if (!y_sem || !y_sem.IsMap())
      {
        return fail("nodes[" + id + "].semantic must be a mapping.");
      }

      // semantic の各キーを検証しつつ列挙に変換
      for (auto it = y_sem.begin(); it != y_sem.end(); ++it)
      {
        const std::string key = it->first.as<std::string>();
        if (!lifecycle_keys.count(key))
        {
          return fail("nodes[" + id + "].semantic has unknown key '" + key + "' not in lifecyclenodes.");
        }
        if (!it->second.IsScalar())
        {
          return fail("nodes[" + id + "].semantic['" + key + "'] must be scalar.");
        }
        SemanticState::State st;
        if (!parse_state(it->second.as<std::string>(), st))
        {
          return fail("nodes[" + id + "].semantic['" + key + "'] has invalid state.");
        }
        tmp.semantic.semantic_state[key] = st;
      }

      // すべての lifecyclenodes のキーが semantic に現れているか（完全指定）をチェック
      if (tmp.semantic.semantic_state.size() != lifecycle_keys.size())
      {
        return fail("nodes[" + id + "].semantic keys must match lifecyclenodes keys.");
      }

      tmp_by_id.emplace(id, std::move(tmp));
    }

    // edges: from/to の存在を検証しつつ、各 from の next に recipe を追加
    const YAML::Node y_edges = root["edges"];
    if (!y_edges || !y_edges.IsSequence())
    {
      return fail("'edges' must exist and be a sequence.");
    }

    for (std::size_t i = 0; i < y_edges.size(); ++i)
    {
      const YAML::Node ye = y_edges[i];
      if (!ye.IsMap())
      {
        return fail("edges[" + std::to_string(i) + "] must be a mapping.");
      }

      const YAML::Node y_from = ye["from"];
      const YAML::Node y_to = ye["to"];
      if (!y_from || !y_from.IsScalar() || !y_to || !y_to.IsScalar())
      {
        return fail("edges[" + std::to_string(i) + "] requires scalar 'from' and 'to'.");
      }
      const std::string from_id = y_from.as<std::string>();
      const std::string to_id = y_to.as<std::string>();

      auto it_from = tmp_by_id.find(from_id);
      if (it_from == tmp_by_id.end())
      {
        return fail("edges[" + std::to_string(i) + "].from '" + from_id + "' not found in nodes.");
      }
      if (!tmp_by_id.count(to_id))
      {
        return fail("edges[" + std::to_string(i) + "].to '" + to_id + "' not found in nodes.");
      }

      // レシピの読み取り（綴りの揺れを許容）
      YAML::Node y_recipe = ye["transitionrecipe"];
      if (!y_recipe)
        y_recipe = ye["trantisionrecipe"]; // YAML サンプルの typo 吸収
      TransitionRecipe recipe;

      if (y_recipe && y_recipe.IsMap())
      {
        YAML::Node y_steps = y_recipe["actionsteps"];
        if (!y_steps)
          y_steps = y_recipe["actionteps"]; // こちらも typo 吸収

        if (y_steps && y_steps.IsSequence())
        {
          recipe.steps.reserve(y_steps.size());
          for (std::size_t si = 0; si < y_steps.size(); ++si)
          {
            const YAML::Node ys = y_steps[si];
            if (!ys.IsMap())
            {
              return fail("edges[" + std::to_string(i) + "].recipe.steps[" + std::to_string(si) + "] must be a mapping.");
            }
            const YAML::Node y_key = ys["target_key"];
            const YAML::Node y_op = ys["operation"];
            const YAML::Node y_tout = ys["timeout_s"];
            if (!y_key || !y_key.IsScalar() || !y_op || !y_op.IsScalar())
            {
              return fail("edges[" + std::to_string(i) + "].recipe.steps[" + std::to_string(si) + "] requires 'target_key' and 'operation'.");
            }
            const std::string target_key = y_key.as<std::string>();
            if (!lifecycle_keys.count(target_key))
            {
              return fail("edges[" + std::to_string(i) + "].recipe.steps[" + std::to_string(si) + "].target_key '" + target_key + "' not in lifecyclenodes.");
            }
            ActionStep step;
            step.target_key = target_key;
            step.operation = y_op.as<std::string>();
            step.timeout_s = as_double_or(y_tout, 0.0);
            recipe.steps.push_back(std::move(step));
          }
        }
        // steps が無くても空レシピとして許容
      }

      // from ノードの next に to_id -> recipe を登録
      auto &from_tmp = it_from->second;
      if (from_tmp.next.count(to_id))
      {
        return fail("Duplicate edge from '" + from_id + "' to '" + to_id + "'.");
      }
      from_tmp.next.emplace(to_id, std::move(recipe));
    }

    // 最後に Graph を構築。TmpNode から Node を生成してベクタへ。
    std::vector<Node> nodes;
    nodes.reserve(tmp_by_id.size());
    for (auto &kv : tmp_by_id)
    {
      TmpNode &t = kv.second;
      nodes.emplace_back(t.id,
                         std::move(t.next),
                         std::move(t.semantic));
    }

    return Graph(std::move(nodes));
  }

} // namespace localization_switcher