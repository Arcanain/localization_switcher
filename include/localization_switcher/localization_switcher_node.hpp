// localization_switcher_node.hpp

#pragma once

#include <array>
#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/static_transform_broadcaster.h"


// Localization Switcher 
// Nodeクラス（ROSインターフェース部分のみでロジックとは切り離す）

namespace localization_switcher
{

class LocalizationSwitchNode : public rclcpp::Node
{
public:
  LocalizationSwitchNode();
  ~LocalizationSwitchNode() = default;

private:
  // privateは，クラスの外からアクセスできない．
  void timerCallback();

  // 用途によってsubは変わる
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void local_obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void send_static_transform();

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr local_obstacle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // States
  std::array<double, 5> x_;
  std::array<double, 2> goal_;
  std::vector<std::array<double, 2>> obstacle_;

  // DWA param
  std::array<double, 6> kinematic_;
  std::array<double, 4> eval_param_;

  double robot_radius_;
  double obstacle_radius_;

  bool received_obstacles_;
  bool received_goal_;
  bool received_odom_;
};

} // namespace dwa_planner
