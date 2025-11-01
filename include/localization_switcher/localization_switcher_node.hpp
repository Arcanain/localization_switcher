// localization_switcher_node.hpp

#pragma once
#include <memory>
#include <chrono>
#include <string>

// ROS2 library
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "localization_switcher/component/localization_switcher_component.hpp"
#include "localization_switcher/component/common_types.hpp"

namespace localization_switcher
{

class LocalizationSwitchNode : public rclcpp::Node
{
public:
  LocalizationSwitchNode();
  ~LocalizationSwitchNode() = default;

private:
  // === Core Timer Loop ===
  void timerCallback();  // 定期的にtickを呼び出す

  // === ROS Subscriber Callbacks ===
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  // === Lifecycle Management ===
  void pollManagedStates();  // 各Lifecycleノードのactive状態を取得
  void executeTransitionRecipe(const TransitionRecipe &recipe);
  void notifyTransitionResult(const ExecResult &result,
                              const std::chrono::steady_clock::time_point &now);

  // === Internal Helpers ===
  WorldState buildWorldState() const;  // トピックデータを整形
  ManagedState buildManagedState() const;  // ライフサイクル状態を整形

private:
  // --- Component ---
  std::shared_ptr<LocalizationSwitcherComponent> component_;

  // --- ROS Interfaces ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Lifecycle Clients ---
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr gnss_lc_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr emcl_lc_client_;

  // --- Node State (data cache) ---
  WorldState world_;
  ManagedState managed_;

  bool executing_transition_{false};
  std::chrono::steady_clock::time_point last_tick_time_;
};

}  // namespace localization_switcher
