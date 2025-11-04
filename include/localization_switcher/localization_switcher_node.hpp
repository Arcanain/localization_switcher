// localization_switcher_node.hpp

#pragma once
#include <memory>
#include <chrono>
#include <string>

// ROS2 library
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// Lifecycle library
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
// Localization Switcher library
#include "localization_switcher/localization_switcher_component.hpp"
#include "localization_switcher/common_types.hpp"

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
  void fixCallback(     const sensor_msgs::msg::NavSatFix::SharedPtr     fixmsg);
  void gnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr gnssmsg);

  // === Lifecycle Management ===
  SemanticState pollSemanticStates();  // 各Lifecycleノードのactive状態を取得
  void executeTransitionRecipe(const TransitionRecipe &recipe);

  // === Internal Helpers ===
  WorldState    buildWorldState() const;  // トピックデータを整形
  SemanticState buildSemanticState() const;  // ライフサイクル状態を整形


  // --- ROS Interfaces ---
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr     fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Lifecycle Clients ---
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr gnss_lc_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr emcl_lc_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr gnss_get_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr emcl_get_client_;

  // --- Node State (data cache) ---
  WorldState my_current_world_;

  // --- Component ---
  std::shared_ptr<LocalizationSwitcherComponent> component_;
  std::atomic<bool> executing_transition_{false};

  std::chrono::steady_clock::time_point last_tick_time_;
};

}  // namespace localization_switcher
