// src/localization_switcher_component.cpp
#include "localization_switcher/localization_switcher_component.hpp"
#include "localization_switcher/internal/transition_decider.hpp"
#include "localization_switcher/internal/graph_builder.hpp"
#include <cmath>
#include <utility>

namespace
{
  // 完全一致で最初のものを返す素朴な規則。
  // 将来、優先キーや重み付けなどに変えるなら、この関数だけを書き換えればよい。
  const localization_switcher::Node *
  select_by_semantics(const localization_switcher::SemanticState &s,
                      const std::vector<localization_switcher::Node> &nodes)
  {
    for (const auto &n : nodes) if (s == n.semantic()) return &n;
    return nullptr;
  }
} // anonymous

namespace localization_switcher
{
  // コンストラクタ
  LocalizationSwitcherComponent::LocalizationSwitcherComponent(const std::string &yaml_path)
      : yaml_path_(yaml_path),
        current_node_(nullptr)
  {
    // 内部クラスのインスタンス作成
    decider_ = std::make_unique<TransitionDecider>(*this);
    if (!initialize_(yaml_path_)) std::cerr << "Warning: Failed to initialize LocalizationSwitcherComponent" << std::endl;
  }

  // デストラクタ
  LocalizationSwitcherComponent::~LocalizationSwitcherComponent() = default;

  bool LocalizationSwitcherComponent::initialize_(const std::string &yaml_path)
  {
    yaml_path_ = yaml_path;

    // GraphBuilderを使ってYAMLからGraphを構築
    auto graph_opt = GraphBuilder::build_from_yaml(yaml_path_);
    if (!graph_opt.has_value())
    {
      std::cerr << "Failed to build graph from YAML: " << yaml_path_ << std::endl;
      return false;
    }

    graph_ = std::move(graph_opt.value());
    return true;
  }

  const SemanticState &LocalizationSwitcherComponent::current_semantic() const noexcept { return current_semantic_; }

  std::chrono::system_clock::time_point LocalizationSwitcherComponent::current_stamp() const noexcept { return current_stamp_; }

  std::optional<TransitionRecipe> LocalizationSwitcherComponent::decide(
      const WorldState &world,
      const SemanticState &semantic,
      TimePoint stamp)
  {
    // セマンティック状態とタイムスタンプの更新
    current_semantic_ = semantic;
    current_stamp_ = (stamp.time_since_epoch().count() == 0)
                        ? std::chrono::system_clock::now()
                        : stamp;

    // 現在のノードを取得
    current_node_ = graph_.get_current_node(current_semantic);
    // current_node_ = graph_.get_current_node(semantic, &select_by_semantics);
    
    if (current_node_ == nullptr) return std::nullopt;

    // 内部クラスに遷移判定を委譲
    return decider_->decide_transition(world);
  }

} // namespace localization_switcher