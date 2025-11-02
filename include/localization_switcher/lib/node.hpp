#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include "localization_switcher/component/common_types.hpp"

namespace localization_switcher
{

    class Node
    {
    public:
        Node(std::string id,                                        // node_id
            std::unordered_map<std::string, TransitionRecipe> next, // to_id -> recipe
            SemanticState semantic                                  // nodeに対応するlifecycleの状態の組み合わせ
        );


        const std::string &id() const;
        std::vector<std::string> next_ids() const;
        const TransitionRecipe* recipe_to(const std::string& to_id) const noexcept; 
        const SemanticState& semantic() const noexcept;

    private:
        std::string id_;
        std::unordered_map<std::string, TransitionRecipe> next_;
        SemanticState semantic_;
    };

} // namespace localization_switcher
