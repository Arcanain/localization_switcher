// localization_swithcer_component.hpp

#pragma once
#include "rclcpp/time.hpp"

namespace localization_switcher
{



// 自己位置推定手法切り替えのロジック(静的メソッドのみ，宣言のみ)

/*
ノードの機能：
  - サブスクライバから外界の状態を受け取る
    自己位置推定の結果，推定の信頼度
  - 現在の自己位置推定の手法が何か→現在の状態を監視する
  - LocalizationSwicherのDecisionの返り値？に応じて，自己位置推定の手法を切り替える

LcalizationSwitcerの機能：


①emclがactiveな時にgnssに遷移する条件を満たしたらtrueを返す
  実装はcppが行うのでここでは宣言のみ
  返り値はbool, 引数の設計が悩みどころだが，fix値（str?）, gnss poseを受け取ることを考える
  そのまま受け渡すとrosに依存してしまうので，ここでは依存しないようにstructで定義する
  timestampも必要
  例)
struct PoseWithCovarianceStamped
{
  int sec;
  int nsec;
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
  double covariance[36];
};  
みたいな感じ？


②gnssがactiveな時にemclに遷移する条件を満たしたらtrueを返す
  実装はcppが行うのでここでは宣言のみ
　  返り値はbool, 引数の設計が悩みどころだが，str(これはfix値), gnss poseを受け取ることを考える

③切り替え条件はあらかじめ何らかの方法で表現し，yamlから読み込む
  例)閾値を超えたら，ある時間以上継続したら，など
  条件はかなり複雑になるはず．例えばemcl -> gnss, gnss->emclで閾値が異なるなど
  それぞれのロジックをじっそうするが，その時のパラメータはyamlから読み込む
  2つの遷移によってパラメータが異なる可能性を考えると，どのようにyamlのデータ表現を設計するかが悩みどころ





*/

struct Waypoint {
  double x;
  double y;
  double radius;
};


class LocalizationSwitcherComponent
{
public:
  LocalizationSwitcherComponent();

  
  void setParameters(
    const std::vector<Waypoint> & gnss_activation_waypoints,
    const std::vector<Waypoint> & emcl_activation_waypoints,
    double duration_to_gnss, double duration_to_emcl,
    double target_radius);

  // --- 意思決定メソッド ---
  bool shouldSwitchToGnss(const rclcpp::Time & current_time);
  bool shouldSwitchToEmcl(const rclcpp::Time & current_time);
  void reset();

private:
  // --- 設定値 ---
  double duration_to_gnss_; // GNSSに切り替えるまでの継続時間
  double duration_to_emcl_; // EMCLに切り替えるまでの継続時間
  double target_radius_;    // ウェイポイントに到達したとみなす半径(m)

  std::vector<Waypoint> gnss_activation_waypoints_; // GNSSに切り替えるためのウェイポイント群
  std::vector<Waypoint> emcl_activation_waypoints_; // EMCLに切り替えるためのウェイポイント群

  size_t gnss_waypoint_index_ = 0; // gnss開始ポイントのインデックス size_tは符号なし整数の型
  size_t emcl_waypoint_index_ = 0; // emcl開始ポイントのインデックス

};

  // 
  rclcpp::Time first_gnss_ok_time_;
  rclcpp::Time first_gnss_lost_time_;
  double current_x_;
  double current_y_;
  bool current_fix_ok_;
};
} // namespace localization_switcher

