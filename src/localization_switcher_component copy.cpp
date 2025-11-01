// localization_switcher_component.cpp

#include "localization_switcher/localization_switcher_component.hpp"

#include <cmath>

namespace localization_switcher
{
  // コンストラクタ
  LocalizationSwitcherComponent::LocalizationSwitcherComponent(std::string yaml_path)
  {
    // メンバ変数の初期化
    this->initialize(ymal_path);
    //
  }


  bool LocalizationSwitcherComponent::initialize(std::string yaml_path)
  {
    graph_ = build_graph_from_yaml_(yaml_path);
    return true; // グラフが異常なく構成された時のみtrueを返すようにしたい
  }

  Graph LocalizationSwitcherComponent::build_graph_from_yaml_(const std::string &yaml_path)
  {
    // YAMLファイルを読み込み，Graphオブジェクトを構築する処理を実装する
    // ここでは仮に空のGraphを返す
    Graph graph;
    return graph;// うまく作れなかった場合にfalseも返したい．方法があれば．C#のoutみたいに
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


  TransitionRecipe LocalizationSwitcherComponent::decide_transition_(const WorldState world_state, const SemanticState &semantic, TimePoint stamp)
  {
    // 最後にtickが呼ばれた時のsemanticと時刻を更新
    last_semantic_ = semantic;
    last_stamp_ = stamp;

    // 現在ノードをグラフから取得
    if (current_node_ == nullptr)
    {
      current_node_ = graph_.get_current_node(semantic);
      // current_node = graph_.get_current_node(semantic, decide_with_priority);
      if (current_node_ == nullptr)
      {
        retuen lllll; 
        //throw std::runtime_error("Current node could not be determined from the graph.");
      }
    }

    // 以下遷移の有無などを判断して返す
    /*
    判断に必要な情報は？
    current_node_ →現在のid,現在のSemanticState(つまり状態),そのidのノードが遷移できるnodeに遷移するためのTransitionRecipe
    world_state → x,y,fix
    あとは何かしら時系列情報を持たせる場合はなにかキューが必要かもしれない，，

    どちらにしても現在の稼働状態と，現在のマシンの状態　（と必要であればそれらの履歴）があれば判断できるか

    

    */

    


    return recipe;
  }
} // namespace localization_switcher