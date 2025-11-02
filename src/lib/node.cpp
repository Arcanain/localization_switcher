#include "localization_switcher/lib/node.hpp"
#include <utility> // std::move

namespace localization_switcher
{

    Node::Node( std::string id,
                std::unordered_map<std::string, TransitionRecipe> next,
                SemanticState semantic)
        : id_(std::move(id)), next_(std::move(next)), semantic_(std::move(semantic)) {}

    const std::string &Node::id() const noexcept { return id_; }

    std::vector<std::string> Node::next_ids() const
    {
        std::vector<std::string> keys;
        keys.reserve(next_.size());
        for (const auto &kv : next_)
            keys.push_back(kv.first);
        return keys;
    }

    const TransitionRecipe *Node::recipe_to(const std::string &to_id) const noexcept
    {
        auto it = next_.find(to_id);
        return it == next_.end() ? nullptr : &it->second;
    }

    const SemanticState &Node::semantic() const noexcept { return semantic_; }

} // namespace localization_switcher
