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

    std::optional<const TransitionRecipe*> LocalizationSwitcherComponent::TransitionDecider::decide_transition(
        WorldState current_world)
    {
        // component_のpublicメンバやfriend経由でprivateメンバにアクセス可能
        if (switch_point_.empty() || current_point_index_ >= static_cast<int>(switch_point_.size())) {
            return std::nullopt;
        }


        const auto& [target_state, to_id] = switch_point_[current_point_index_];

        // 標準出力
        // デバッグ出力（構造化束縛した変数をそのまま使う）
        std::cout   << "[TransitionDecider] idx=" << current_point_index_
                    << " target=(" << target_state.x << ", " << target_state.y << ")"
                    << " to=" << to_id
                    << " current=" << component_.current_node_->id()
                    << std::endl;


        if (component_.current_node_ == nullptr)        return std::nullopt;
        if (component_.current_node_->id() == to_id) return std::nullopt;
    


        //if (component_.current_node_->node_id == "GNSS_ONLY") 
        if (to_id == "EMCL_ONLY") //本来はcurrent_node_-> node_id と to_idのりょうほうをつかうべき
        {
            // current_node_-> id がgnssの場合
            if (shouldSwitchToEmcl_(std::chrono::steady_clock::now(), current_world)) return component_.current_node_->recipe_to("EMCL_ONLY");
        }


        //if (component_.current_node_->node_id == "EMCL_ONLY")
        if (to_id == "GNSS_ONLY")
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

        const WorldState& target = std::get<0>(switch_point_[current_point_index_]);

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

        const WorldState& target = std::get<0>(switch_point_[current_point_index_]);

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


void LocalizationSwitcherComponent::TransitionDecider::load_parameters_from_yaml(
    const std::string& yaml_path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } 
    catch (const std::exception& e) {
        std::cerr << "[TransitionDecider] YAML load failed: " << e.what() << std::endl;
        return;
    }

    const YAML::Node dec = root["decider"];
    if (!dec || !dec.IsMap()) {
        std::cerr << "[TransitionDecider] 'decider' section missing." << std::endl;
        return;
    }

    const YAML::Node pts = dec["switch_points"];
    if (!pts || !pts.IsSequence()) {
        std::cerr << "[TransitionDecider] 'decider.switch_points' must be a sequence." << std::endl;
        return;
    }

    switch_point_.clear();
    switch_point_.reserve(pts.size());

    for (std::size_t i = 0; i < pts.size(); ++i) {
        const YAML::Node p = pts[i];
        if (!p.IsMap()) {
            std::cerr << "[TransitionDecider] switch_points[" << i << "] must be a map." << std::endl;
            continue;
            }

        if (!p["x"] || !p["y"] || !p["to"]) {
            std::cerr << "[TransitionDecider] switch_points[" << i << "] requires x, y, to." << std::endl;
            continue;
        }

        const double x = p["x"].as<double>();
        const double y = p["y"].as<double>();
        const std::string to_id = p["to"].as<std::string>();

        switch_point_.emplace_back(WorldState{x, y, false}, to_id);
    }

    current_point_index_ = 0;
    gnss_transition_timer_start_ = std::chrono::steady_clock::time_point{};
    emcl_transition_timer_start_ = std::chrono::steady_clock::time_point{};
    reach_flag_ = false;
    duration_to_gnss_ = 0.0;
    duration_to_emcl_ = 0.0;
    d_th_to_gnss_ = 5.0;
    d_th_to_emcl_ = 5.0;
    current_point_index_ = 0;

    if (switch_point_.empty())
        std::cerr << "[TransitionDecider] No valid switch_points loaded." << std::endl;
}

} // namespace localization_switcher