#include "localization_switcher/internal/transition_decider.hpp"


namespace localization_switcher
{
    // コンストラクタ
    LocalizationSwitcherComponent::TransitionDecider::TransitionDecider(
        LocalizationSwitcherComponent &comp,
        const std::string& yaml_path)
        : component_(comp)
    {
        load_parameters_from_yaml(yaml_path);
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
            if (elapsed >= std::chrono::duration<double>(duration_to_gnss_)) {
                ++current_point_index_;       // 次の目標に進む
                reach_flag_ = false;          // 状態をリセット
                return true;
            }
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
            if (elapsed >= std::chrono::duration<double>(duration_to_emcl_)) {
                ++current_point_index_;       // 次の目標に進む
                reach_flag_ = false;          // 状態をリセット
                return true;
            }
        }
        else
        {
            // fixでなければリセット
            emcl_transition_timer_start_ = {};
        }
        return false;
    }


    // TransitionDecider のメンバ関数として実装
// yaml: 
// decider:
//   switch_points:
//     - { x: 10.0, y: 5.0 }
//     - { x: 25.0, y: 8.0 }
//     - { x: 50.0, y: -3.0 }
void LocalizationSwitcherComponent::TransitionDecider::load_parameters_from_yaml(
    const std::string& yaml_path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        std::cerr << "[TransitionDecider] YAML load failed: " << e.what() << std::endl;
        return;
    }

    const YAML::Node decider = root["decider"];
    if (!decider || !decider.IsMap()) {
        std::cerr << "[TransitionDecider] 'decider' section is missing or not a map." << std::endl;
        return;
    }

    const YAML::Node switch_points = decider["switch_points"];
    if (!switch_points || !switch_points.IsSequence()) {
        std::cerr << "[TransitionDecider] 'decider.switch_points' must be a sequence." << std::endl;
        return;
    }

    switch_point_.clear();
    switch_point_.reserve(switch_points.size());

    for (std::size_t i = 0; i < switch_points.size(); ++i) {
        const YAML::Node p = switch_points[i];
        if (!p.IsMap()) {
            std::cerr << "[TransitionDecider] switch_points[" << i << "] must be a map." << std::endl;
            continue;
        }
        const YAML::Node xn = p["x"];
        const YAML::Node yn = p["y"];
        if (!xn || !xn.IsScalar() || !yn || !yn.IsScalar()) {
            std::cerr << "[TransitionDecider] switch_points[" << i << "] requires scalar 'x' and 'y'." << std::endl;
            continue;
        }

        const double x = xn.as<double>();
        const double y = yn.as<double>();

        // fix_status はここでは未使用のためダミー false を格納
        switch_point_.push_back(WorldState{ x, y, /*fix_status=*/false });
    }

    // 読み直し時の状態をクリア
    current_point_index_ = 0;
    reach_flag_ = false;
    gnss_transition_timer_start_ = std::chrono::steady_clock::time_point{};
    emcl_transition_timer_start_ = std::chrono::steady_clock::time_point{};
    reach_flag_ = false;
    duration_to_gnss_ = 0.0;
    duration_to_emcl_ = 0.0;
    d_th_to_gnss_ = 5.0;
    d_th_to_emcl_ = 5.0;
    current_point_index_ = 0;

    if (switch_point_.empty()) {
        std::cerr << "[TransitionDecider] No valid switch_points loaded." << std::endl;
    }
}

} // namespace localization_switcher