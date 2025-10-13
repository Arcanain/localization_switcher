// localization_swithcer_component.hpp

#pragma once
#include <vector>
#include <chrono>

namespace localization_switcher
{

  // 自己位置推定手法切り替えのロジック(宣言のみ)
  /*
  Lifecycle nodeを管理するにあたってロボットのマシンを管理する必要があるのでインスタンス化することにした
  */

  struct Waypoint
  {
    double x;
    double y;
    double radius;
  };

  class LocalizationSwitcherComponent
  {
  public:
    LocalizationSwitcherComponent();

    // 初期化用メソッド
    void setParameters(
        const std::vector<Waypoint> &gnss_activation_points, // GNSSが開始する地点
        const std::vector<Waypoint> &emcl_activation_points, // EMCLが開始する地点
        double duration_to_gnss, double duration_to_emcl);
    void initialize();

    // --- 状態更新用メソッド ---
    void updateGnssPose(double x, double y);
    void updateFixStatus(bool fix_status); // TODO tipicを受け取るので，ここはstrかもしれない．あとで確認

    // --- 意思決定メソッド ---
    bool shouldSwitchToGnss(const std::chrono::steady_clock::time_point &current_time);
    bool shouldSwitchToEmcl(const std::chrono::steady_clock::time_point &current_time);

    // --- 状態リセット用メソッド ---

    void resetGnssActivation(); // 機能？
    void resetEmclActivation(); // 機能？

  private:

    // --- 設定値 (YAMLから読み込む) ---
    double duration_to_gnss_;
    double duration_to_emcl_;
    std::vector<Waypoint> gnss_activation_points_;
    std::vector<Waypoint> emcl_activation_points_;

    // --- 状態変数 ---
    size_t gnss_waypoint_index_;
    size_t emcl_waypoint_index_;

    // 遷移地点に到達した時刻を保存する変数
    std::chrono::steady_clock::time_point gnss_transition_timer_start_;
    std::chrono::steady_clock::time_point emcl_transition_timer_start_;

    double current_x_ ;
    double current_y_ ;
    bool current_fix_status_ ;
    Waypoint current_target_wp_{0.0, 0.0, 0.0}; // ゼロ初期化してOK?
    bool reach_flag_ = false;                   // GNSS切り替えゾーンに到達したか
  };

} // namespace localization_switcher
