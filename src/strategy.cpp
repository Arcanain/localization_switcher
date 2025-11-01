#include "localization_switcher/component/strategy.hpp"

namespace localization_switcher
{

    const Node *decide_with_priority(
        const SemanticState &s,
        const std::vector<Node> &candidates)
    {
        const Node *best = nullptr;
        for (const Node &n : candidates)
        { // ここが違う：const Node& n
            if (s == n.semantic())
            {
                best = &n; // &n でポインタを返す
                break;
            }
        }
        return best;
    }

} // namespace localization_switcher
