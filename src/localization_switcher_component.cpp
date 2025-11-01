// localization_switcher/component/localization_switcher_component.cpp
#include "localization_switcher/component/localization_switcher_component.hpp"
#include <cmath>
#include <utility>

namespace localization_switcher
{

  LocalizationSwitcherComponent::LocalizationSwitcherComponent(const std::string &yaml_path)
      : yaml_path_(yaml_path)
  {
    initialize_(yaml_path_);
  }

  bool LocalizationSwitcherComponent::initialize_(const std::string &yaml_path)
  {
    yaml_path_ = yaml_path;
    graph_ = build_graph_from_yaml_(yaml_path_);
    return true; // 将来: 失敗検知に置き換え
  }

  Graph LocalizationSwitcherComponent::build_graph_from_yaml_(const std::string &yaml_path)
  {
    (void)yaml_path;
    return Graph{}; // TODO: yaml-cpp 等で実装
  }

  const SemanticState &LocalizationSwitcherComponent::last_semantic() const noexcept
  {
    return last_semantic_;
  }

  //LocalizationSwitcherComponent::TimePoint
  
  LocalizationSwitcherComponent::last_stamp() const noexcept
  {
    return last_stamp_;
  }

  TransitionRecipe LocalizationSwitcherComponent::decide(
      const WorldState &world,
      const SemanticState &semantic,
      TimePoint stamp)
  {
    return decide_transition_(world, semantic, /*solver=*/nullptr, stamp);
  }

  TransitionRecipe LocalizationSwitcherComponent::decide_transition_(
      const WorldState &world,
      const SemanticState &semantic,
      DecisionSolverPtr solver,
      TimePoint stamp)
  {
    last_semantic_ = semantic;
    last_stamp_ = (stamp.time_since_epoch().count() == 0) ? Clock::now() : stamp;

    // WorldState → 内部キャッシュ（あなたの WorldState に合わせて置換）
    current_x_ = world.x;
    current_y_ = world.y;
    current_fix_status_ = world.fix_ok;

    // 現在ノードの同定（Graph の通常版。戦略付き NodeSolver を使う設計なら差し替え可）
    if (const Node *n = graph_.get_current_node(semantic))
    {
      current_node_ = n;
    }
    else
    {
      current_node_ = nullptr;
      return std::nullopt;
    }

    // 戦略ありなら全面委譲
    if (solver)
    {
      return solver(world, semantic, *current_node_, graph_, last_stamp_);
    }

    // ここから内蔵ロジック（最小限の例。不要なら常に nullopt を返すだけでよい）
    const auto now_steady = std::chrono::steady_clock::now();
    const bool to_gnss = shouldSwitchToGnss_(now_steady);
    const bool to_emcl = shouldSwitchToEmcl_(now_steady);

    if (!to_gnss && !to_emcl)
      return std::nullopt;

    const std::string to_id = to_gnss ? "GNSS_ONLY" : "EMCL_ONLY";
    if (const TransitionRecipe *r = current_node_->recipe_to(to_id))
    {
      return *r;
    }
    return std::nullopt;
  }

  bool LocalizationSwitcherComponent::isInActivationZone_(const Waypoint &wp)
  {
    const double dx = current_x_ - wp.x;
    const double dy = current_y_ - wp.y;
    return std::hypot(dx, dy) <= wp.radius;
  }

  bool LocalizationSwitcherComponent::shouldSwitchToGnss_(const std::chrono::steady_clock::time_point &now)
  {
    if (!reach_flag_)
      reach_flag_ = isInActivationZone_(current_target_wp_);
    if (!reach_flag_)
      return false;

    if (current_fix_status_)
    {
      if (gnss_transition_timer_start_.time_since_epoch().count() == 0)
      {
        gnss_transition_timer_start_ = now;
      }
      const auto elapsed = now - gnss_transition_timer_start_;
      if (elapsed >= std::chrono::duration<double>(duration_to_gnss_))
        return true;
    }
    else
    {
      gnss_transition_timer_start_ = {};
    }
    return false;
  }

  bool LocalizationSwitcherComponent::shouldSwitchToEmcl_(const std::chrono::steady_clock::time_point &now)
  {
    if (!reach_flag_)
      reach_flag_ = isInActivationZone_(current_target_wp_);
    if (!reach_flag_)
      return false;

    if (current_fix_status_)
    {
      if (emcl_transition_timer_start_.time_since_epoch().count() == 0)
      {
        emcl_transition_timer_start_ = now;
      }
      const auto elapsed = now - emcl_transition_timer_start_;
      if (elapsed >= std::chrono::duration<double>(duration_to_emcl_))
        return true;
    }
    else
    {
      emcl_transition_timer_start_ = {};
    }
    return false;
  }

} // namespace localization_switcher
