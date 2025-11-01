// component.hpp
#pragma once
#include <string>
#include <chrono>
#include "localization_switcher/component/graph.hpp"
#include "localization_switcher/component/common_types.hpp"
#include "localization_switcher/strategy.hpp"

namespace localization_switcher
{

  class LocalizationSwitcherComponent
  {
  public:
    using Clock = std::chrono::system_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    // コンストラクタ
    explicit LocalizationSwitcherComponent(
        const std::string &yaml_path,
        DecisionSolverPtr solver = nullptr);

    // tick 次のノードを決定する
    // 　
    TransitionRecipe decide(const WorldState world_state, const SemanticState &semantic, TimePoint stamp = TimePoint{});

    // const Node* current_node() const noexcept;// 不要？
    // std::string current_node_id() const;これもpublicで使うことがない→なぜならrosはnodeについて知らないから．
    const SemanticState &last_semantic() const noexcept; // 　最後にtickが呼ばれた時のsemanticを返す
    TimePoint last_stamp() const noexcept;               // 　最後にtickが呼ばれた時刻を返す

  private:
    // YAML ファイルから初期化する（ROS 側から呼ばれる）
    bool initialize_(const std::string &yaml_path);
    // YAML 読み込みと Graph 構築を内部で完結させるプライベート関数
    Graph build_graph_from_yaml_(const std::string &yaml_path);
    TransitionRecipe decide_transition_(const WorldState world_state, const SemanticState &semantic, TimePoint stamp);

    Graph graph_;
    const Node *current_node_{nullptr};
    SemanticState last_semantic_{};
    TimePoint last_stamp_{};
    // bool initialized_{false};
  };

} // namespace localization_switcher
