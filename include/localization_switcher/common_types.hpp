#pragma once
#include <string>
#include <vector>
#include <unordered_map>

namespace localization_switcher
{

    // 外界の観測を集約（ROSメッセージ非依存）
    struct WorldState
    {
        double x;
        double y;
        bool fix_status; // GNSSがfixしているか
        // 必要に応じて拡張（精度・速度など）
    };

    // ライフサイクルノードの状態の直積でモードを表現する
    struct SemanticState
    {
        enum class State : std::uint8_t
        {
            UNKNOWN = 0,
            UNCONFIGURED = 1,
            INACTIVE = 2,
            ACTIVE = 3,
            FINALIZED = 4
        };

        std::unordered_map<std::string, State> semantic_state;
        bool operator==(const SemanticState &other) const noexcept
        {
            return semantic_state == other.semantic_state;
        }
    };

    // アクション手順（Lifecycleノード操作）
    struct ActionStep
    {
        std::string target_node_name; // "gnss" or "emcl"
        std::string operation;        // "configure" / "activate" / "deactivate"
        double timeout_s{5.0};
        int retry{0};
    };

    // 遷移レシピ（ひとつのエッジに対応）
    struct TransitionRecipe
    {
        std::vector<ActionStep> steps;
        std::string description;
    };

    // 意思決定結果（Component→Node）
    struct Decision
    {
        std::string from; // 現在モード
        std::string to;   // 目標モード
        TransitionRecipe recipe;
        std::string reason; // ログ・診断用
    };

    // 実行結果NodeからComponentへ返す
    struct ExecResult
    {
        bool success;
        std::string message;
    };

} // namespace localization_switcher
