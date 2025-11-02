#include "localization_switcher/internal/transition_decider.hpp"
#include <cmath>

namespace localization_switcher
{
    // コンストラクタ
    LocalizationSwitcherComponent::TransitionDecider::TransitionDecider(
        LocalizationSwitcherComponent &comp)
        : component_(comp)
    {
        reach_flag_ = false;
        duration_to_gnss_ = 0.0;
        duration_to_emcl_ = 0.0;
        d_th_to_gnss_ = 5.0;
        d_th_to_emcl_ = 5.0;
        current_point_index_ = 0;
    }

    std::optional<TransitionRecipe> LocalizationSwitcherComponent::TransitionDecider::decide_transition(
        WorldState current_world)
    {
        // component_のpublicメンバやfriend経由でprivateメンバにアクセス可能

        if (component_.current_node_ == nullptr) return std::nullopt;

        if (component_.current_node_->node_id == "GNSS_ONLY")
        {
            // current_node_-> id がgnssの場合
            if (shouldSwitchToEmcl_(std::chrono::steady_clock::now(), current_world)) return component_.current_node_->recipe_to("EMCL_ONLY");
        }

        if (component_.current_node_->node_id == "EMCL_ONLY")
        {
            // current_node_-> id がgnssの場合
            if (shouldSwitchToGnss_(std::chrono::steady_clock::now(), current_world)) return component_.current_node_->recipe_to("GNSS_ONLY");
        }

        return std::nullopt;
    }

    bool LocalizationSwitcherComponent::TransitionDecider::isLessThanThreshold_(
        const WorldState target,
        const WorldState current,
        const double threshold) const
    {
        return std::hypot(target.x - current.x, target.y - current.y) <= threshold;
    }

    bool LocalizationSwitcherComponent::TransitionDecider::shouldSwitchToGnss_(
        const std::chrono::steady_clock::time_point &now,
        WorldState current_world)
    {
        if (current_point_index_ >= static_cast<int>(switch_point_.size())) return false;

        WorldState target = switch_point_[current_point_index_]; // switch pointを取得

        if (!reach_flag_) reach_flag_ = isLessThanThreshold_(target, current_world, d_th_to_gnss_);
        
        if (!reach_flag_) return false;

        if (current_world.fix_status)
        {
            // fixしていて，タイマーが開始していない場合→スタートする
            if (gnss_transition_timer_start_.time_since_epoch().count() == 0) gnss_transition_timer_start_ = now;

            // 経過時間が閾値を超えたらスイッチする
            const auto elapsed = now - gnss_transition_timer_start_;
            if (elapsed >= std::chrono::duration<double>(duration_to_gnss_)) return true;
        }
        else
        {
            // fixでなければリセット
            gnss_transition_timer_start_ = {};
        }
        return false;
    }

    bool LocalizationSwitcherComponent::TransitionDecider::shouldSwitchToEmcl_(
        const std::chrono::steady_clock::time_point &now,
        WorldState current_world)
    {
        if (current_point_index_ >= static_cast<int>(switch_point_.size())) return false;

        WorldState target = switch_point_[current_point_index_]; // switch pointを取得

        if (!reach_flag_) reach_flag_ = isLessThanThreshold_(target, current_world, d_th_to_emcl_);
        
        if (!reach_flag_) return false;

        if (current_world.fix_status)
        {
            // fixしていて，タイマーが開始していない場合→スタートする
            if (emcl_transition_timer_start_.time_since_epoch().count() == 0) emcl_transition_timer_start_ = now;

            // 経過時間が閾値を超えたらスイッチする
            const auto elapsed = now - emcl_transition_timer_start_;
            if (elapsed >= std::chrono::duration<double>(duration_to_emcl_)) return true;
        }
        else
        {
            // fixでなければリセット
            emcl_transition_timer_start_ = {};
        }
        return false;
    }

} // namespace localization_switcher