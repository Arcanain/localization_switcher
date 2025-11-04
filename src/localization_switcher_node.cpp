// localization_switcher_node.cpp

// src/localization_switcher_node.cpp
#include "localization_switcher/localization_switcher_node.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <future>
#include <algorithm> // std::max
#include <chrono>

// 実ノード名は当面固定（YAMLに合わせる）
static constexpr const char *GNSS_NODE = "gnss";
static constexpr const char *EMCL_NODE = "emcl2";

namespace localization_switcher
{
    // コンストラクタ
    LocalizationSwitchNode::LocalizationSwitchNode()
        : rclcpp::Node("localization_switch_node")
    {
        // --- Parameters ---
        const std::string graph_yaml_default = this->declare_parameter<std::string>("graph_yaml_path", "");
        const std::string decider_yaml_default = this->declare_parameter<std::string>("decider_yaml_path", "");
        const double tick_hz = this->declare_parameter<double>("tick_hz", 2.0); // [Hz]

        std::string graph_yaml = graph_yaml_default;
        std::string decider_yaml = decider_yaml_default;
        //(void)graph_yaml;
        //(void)decider_yaml; // 未使用警告抑止（実際は空でなければ使う）

        // --- Component 構築 ---
        component_ = std::make_shared<LocalizationSwitcherComponent>(graph_yaml, decider_yaml);

        // --- Subscribers ---
        using std::placeholders::_1;

        fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/ublox_gps_node/fix", 10, std::bind(&LocalizationSwitchNode::fixCallback, this, _1));

        gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/gnss_pose", 10, std::bind(&LocalizationSwitchNode::gnssPoseCallback, this, _1));

        // --- Lifecycle Clients ---
        gnss_lc_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            std::string("/") + GNSS_NODE + "/change_state");
        emcl_lc_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            std::string("/") + EMCL_NODE + "/change_state");
        gnss_get_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
            std::string("/") + GNSS_NODE + "/get_state");
        emcl_get_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
            std::string("/") + EMCL_NODE + "/get_state");

        // --- Timer ---
        auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, tick_hz)); // 周期[s]
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&LocalizationSwitchNode::timerCallback, this));

        last_tick_time_ = std::chrono::steady_clock::now();
    }

    // === Callbacks ===
    // Fix ステータス更新
    void LocalizationSwitchNode::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fixmsg)
    {
        my_current_world_.fix_status = (fixmsg->status.status == 2);
    }

    // GNSSPose ステータス更新
    void LocalizationSwitchNode::gnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr gnssmsg)
    {
        my_current_world_.x = gnssmsg->pose.position.x;
        my_current_world_.y = gnssmsg->pose.position.y;
    }

    // メイン処理ループ

    void LocalizationSwitchNode::timerCallback()
    {
        // すでに実行中なら軽く弾く（ここでは立てない）
        if (executing_transition_.load(std::memory_order_acquire))
        {
            RCLCPP_DEBUG(this->get_logger(), "transition busy; skip this tick");
            return;
        }

        // 状態取得と判定は普通に走らせる
        const SemanticState current_semantic = pollSemanticStates();
        const auto maybe = component_->decide(  my_current_world_,
                                                current_semantic,
                                                std::chrono::system_clock::now());
        if (!maybe)
        {
            return; // レシピが無いなら何もしない（フラグも立てない）
        }

        // ここで初めて「実行権」を取りに行く：false→true の獲得に成功した一人だけ実行
        bool expected = false;
        if (!executing_transition_.compare_exchange_strong( expected, true,
                                                            std::memory_order_acq_rel))
        {
            return; // 他所がすでに実行開始していた
        }

        // 必ず解除するためのガード
        struct Guard
        {
            std::atomic<bool> &f;
            ~Guard() { f.store(false, std::memory_order_release); }
        } guard{executing_transition_};

        // レシピ実行（同期）
        executeTransitionRecipe(**maybe);
    }

    // === Helpers ===
    // 各ライフサイクルノードの状態をポーリングして managed_ を更新
    SemanticState LocalizationSwitchNode::pollSemanticStates()
    {
        SemanticState semantic;

        auto query_state = [this](const rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr &client,
                                  const char *node_name) -> SemanticState::State
        {
            using ServiceT = lifecycle_msgs::srv::GetState;

            if (!client)
            {
                RCLCPP_WARN(this->get_logger(), "[GetState] client for %s is null", node_name);
                return SemanticState::State::UNKNOWN;
            }

            if (!client->wait_for_service(std::chrono::milliseconds(450)))
            {
                RCLCPP_WARN(this->get_logger(), "[GetState] %s/get_state not available", node_name);
                return SemanticState::State::UNKNOWN;
            }

            auto req = std::make_shared<ServiceT::Request>();
            auto future = client->async_send_request(req);

            auto rc = rclcpp::spin_until_future_complete(
                this->get_node_base_interface(), future, std::chrono::milliseconds(500));

            if (rc != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_WARN(this->get_logger(), "[GetState] %s/get_state request failed", node_name);
                return SemanticState::State::UNKNOWN;
            }

            const auto id = future.get()->current_state.id;
            switch (id)
            {
            case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
                return SemanticState::State::ACTIVE;
            case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
                return SemanticState::State::INACTIVE;
            case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
                return SemanticState::State::UNCONFIGURED;
            default:
                return SemanticState::State::UNKNOWN;
            }
        };

        // 実ノード名（YAMLと対応）
        semantic.semantic_state[GNSS_NODE] = query_state(gnss_get_client_, GNSS_NODE);
        semantic.semantic_state[EMCL_NODE] = query_state(emcl_get_client_, EMCL_NODE);

        return semantic;
    }

    // レシピに従ってライフサイクルノードを操作
    void LocalizationSwitchNode::executeTransitionRecipe(const TransitionRecipe &recipe)
    {
        for (const ActionStep &step : recipe.steps)
        {
            rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client;
            if (step.target_node_name == GNSS_NODE)
                client = gnss_lc_client_;
            else if (step.target_node_name == EMCL_NODE)
                client = emcl_lc_client_;
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unknown node '%s'; skipping", step.target_node_name.c_str());
                continue;
            }

            const double tout_s = (step.timeout_s > 0.0) ? step.timeout_s : 5.0;
            if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(tout_s))))
            {
                RCLCPP_WARN(this->get_logger(), "%s/change_state not available", step.target_node_name.c_str());
                continue;
            }

            auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            lifecycle_msgs::msg::Transition tr;
            if (step.operation == "configure")
                tr.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
            else if (step.operation == "activate")
                tr.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
            else if (step.operation == "deactivate")
                tr.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
            else if (step.operation == "cleanup")
                tr.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
            else if (step.operation == "shutdown")
                tr.id = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unknown operation '%s'; skipping", step.operation.c_str());
                continue;
            }
            req->transition = tr;

            auto fut = client->async_send_request(req);
            auto rc = rclcpp::spin_until_future_complete(
                this->get_node_base_interface(), fut,
                std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(tout_s)));

            if (rc != rclcpp::FutureReturnCode::SUCCESS || !fut.get()->success)
            {
                RCLCPP_WARN(this->get_logger(), "Transition '%s' for %s failed or timed out",
                            step.operation.c_str(), step.target_node_name.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Transition '%s' for %s succeeded",
                            step.operation.c_str(), step.target_node_name.c_str());
            }
        }
    }

} // namespace localization_switcher
