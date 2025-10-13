// localization_switcher_component.cpp

#include "localization_switcher/localization_switcher_component.hpp"
#include <cmath>

namespace localization_switcher
{
  // コンストラクタ
  LocalizationSwitcherComponent::LocalizationSwitcherComponent()
  {
    // メンバ変数の初期化
    this->initialize();
  }

  // --- 初期化用メソッド ---
  void LocalizationSwitcherComponent::setParameters(
      const std::vector<Waypoint> &gnss_activation_points,
      const std::vector<Waypoint> &emcl_activation_points,
      double duration_to_gnss, double duration_to_emcl)
  {
    gnss_activation_points_ = gnss_activation_points;
    emcl_activation_points_ = emcl_activation_points;
    duration_to_gnss_ = duration_to_gnss;
    duration_to_emcl_ = duration_to_emcl;
  }

  void LocalizationSwitcherComponent::initialize()
  {
    gnss_waypoint_index_ = 0;
    emcl_waypoint_index_ = 0;
    reach_flag_ = false;
    reach_flag_ = false;
    gnss_transition_timer_start_ = {};
    emcl_transition_timer_start_ = {};
  }

  void LocalizationSwitcherComponent::updateGnssPose(double x, double y)
  {
    current_x_ = x;
    current_y_ = y;
  }

  void LocalizationSwitcherComponent::updateFixStatus(bool fix_status)
  {
    current_fix_status_ = fix_status;
  }

  void LocalizationSwitcherComponent::updateCurrentTargetWaypoint(const Waypoint &wp)
  {
    current_target_wp_ = wp;
  }

  bool LocalizationSwitcherComponent::isInActivationZone(const Waypoint &wp)
  {
    double distance_to_wp = std::hypot(current_x_ - wp.x, current_y_ - wp.y);
    return (distance_to_wp <= wp.radius);
  }

  // emcl -> gnss への切り替え条件を判定
  bool LocalizationSwitcherComponent::shouldSwitchToGnss(const std::chrono::steady_clock::time_point &current_time)
  {

    // ① 切り替え地点に到達しているか判定する
    // 到達していない時のみ判定する，一度到達したら，切り替えの処理が終わるまでreached_flag trueのままにする
    if (reach_flag_ == false)
    {
      reach_flag_ = isInActivationZone(current_target_wp_);
    }

    // ②到達している時のみ，切り替え判定を行う．
    /*
    この判定ロジックは，reached_flagがtrueの時にだけ入る．reached_flagは，切り替えが終了するまでtrueのままになる．

    初めてreach_flagがtrueになった時刻からタイマーを開始する．

    切り替え判定：
      タイマーが開始している時：
        タイマー開始から duration_to_gnss_ 秒が経過している
          trueを返す
        経過していない
          fix statusが2であればそのままタイマーを継続する
          fix statueが2でない場合→タイマーをリセットする
          falseを返す
    　
      タイマーが開始していない時：
        開始する．
        falseを返す
    */

    if (reach_flag_)
    {
      if (current_fix_status_)
      {
        // タイマーがまだ開始されていなければ、開始する
        if (gnss_transition_timer_start_.time_since_epoch().count() == 0)
        {
          gnss_transition_timer_start_ = current_time;
        }

        // タイマーが開始されている場合、経過時間をチェック
        auto duration = current_time - gnss_transition_timer_start_;
        if (duration >= std::chrono::duration<double>(duration_to_gnss_))
        {
          return true; // 指定時間が経過したので、切り替え条件成立
        }
      }
      else
      {
        // FIX状態が悪化した場合、タイマーをリセットしてやり直し
        gnss_transition_timer_start_ = {};
      }
    }

    return false;
  }

  // gnss -> emcl への切り替え条件を判定
  bool LocalizationSwitcherComponent::shouldSwitchToEmcl(const std::chrono::steady_clock::time_point &current_time)
  {
    // ① 切り替え地点に到達しているか判定する
    if (reach_flag_ == false)
    {
      reach_flag_ = isInActivationZone(current_target_wp_);
    }

    if (reach_flag_)
    {
      if (current_fix_status_)
      {
        if (emcl_transition_timer_start_.time_since_epoch().count() == 0)
        {
          emcl_transition_timer_start_ = current_time;
        }
        auto duration = current_time - emcl_trantision_timer_start_;
        if (duration >= std::chrono::duration<double>(duration_to_emcl_))
        {
          return true;
        }
      }
      else
      {
        emcl_transition_timer_start_ = {};
      }
    }
    return false;
  }

} // namespace localization_switcher