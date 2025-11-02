// include/localization_switcher/localization_switcher_component.hpp
#pragma once
#include <string>
#include <chrono>
#include <optional>
#include <memory>

#include "localization_switcher/lib/graph.hpp"
#include "localization_switcher/lib/node.hpp"
#include "localization_switcher/common_types.hpp"

namespace localization_switcher
{

  class LocalizationSwitcherComponent
  {
  public:
    using Clock = std::chrono::system_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    explicit LocalizationSwitcherComponent(const std::string &yaml_path);
    ~LocalizationSwitcherComponent();

    const SemanticState &current_semantic() const noexcept;
    std::chrono::system_clock::time_point current_stamp() const noexcept;

    std::optional<TransitionRecipe> decide(
        const WorldState &world,
        const SemanticState &semantic,
        TimePoint stamp = TimePoint{});

  private: // private methods
    // 内部クラスの前方宣言
    class TransitionDecider;
    friend class TransitionDecider;

    // メンバ変数
    std::string yaml_path_;
    Graph graph_;
    SemanticState current_semantic_;
    TimePoint current_stamp_;
    const Node *current_node_;

    // 内部クラスのインスタンス
    std::unique_ptr<TransitionDecider> decider_;

    // 初期化関数
    bool initialize_(const std::string &yaml_path);
  };

} // namespace localization_switcher
