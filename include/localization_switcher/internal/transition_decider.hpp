#pragma once
#include "localization_switcher/localization_switcher_component.hpp"
#include <optional>
#include <chrono>
#include <vector>
#include <tuple>
#include <iostream>
#include <cmath>
#include <yaml-cpp/yaml.h>

namespace localization_switcher
{
  // private内部クラスの定義
  class LocalizationSwitcherComponent::TransitionDecider
  {
  public:
    explicit TransitionDecider(LocalizationSwitcherComponent &comp, const std::string& yaml_path);
    
    // 遷移判定のメインロジック
    std::optional<const TransitionRecipe*> decide_transition(WorldState current_world);

  private:
    LocalizationSwitcherComponent &component_;

    // TransitionDecider固有の状態変数
    bool reach_flag_ = false;

    // gsss -> emcl への切り替えポイントに達した時に開始するタイマー
    std::chrono::steady_clock::time_point gnss_transition_timer_start_;
    // emcl -> gnss への切り替えポイントに達した時に開始するタイマー
    std::chrono::steady_clock::time_point emcl_transition_timer_start_;

    double duration_to_gnss_;     // emcl -> gnss 切り替えがtrueになるまでの時間
    double duration_to_emcl_;     // gnss -> emcl 切り替えがtrueになるまでの時間
    double d_th_to_gnss_ ;   // emcl -> gnss ポイントの距離閾値
    double d_th_to_emcl_ ;   // gnss -> emcl ポイントの距離閾値

    std::vector<std::tuple<WorldState, std::string>> switch_point_;    // 切り替えポイントのリスト
    int current_point_index_ ;  // 切り替えポイントのインデックス

    // 判定用のprivateメソッド
    bool isLessThanThreshold_(const WorldState target, const WorldState current, const double threshold) const;
    bool shouldSwitchToGnss_(const std::chrono::steady_clock::time_point &now, WorldState current_world);
    bool shouldSwitchToEmcl_(const std::chrono::steady_clock::time_point &now, WorldState current_world);
    void load_parameters_from_yaml(const std::string& yaml_path);
  };

} // namespace localization_switcher
