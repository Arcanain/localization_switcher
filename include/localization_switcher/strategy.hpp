// include/localization_switcher/strategy.hpp
#pragma once
#include <vector>
#include "localization_switcher/node.hpp"
#include "localization_switcher/common_types.hpp"
// #include "localization_switcher/strategy.hpp"

namespace localization_switcher
{
    // ====== FOR Graph ======
    using NodeSolver = const Node *(const SemanticState &, const std::vector<Node> &);
    using NodeSolverPtr = NodeSolver *;

    // NodeSolver型の関数の実装例
    const Node *decide_with_priority(const SemanticState &s,
                                     const std::vector<const Node *> &nodes);

    // ====== FOR Component ======
    using DecisionSolver = const Decision *(const WorldState &, const SemanticState &);
    using DecisionSolverPtr = DecisionSolver *;
    // example
    const Decision *decide_sample();
    

} // namespace localization_switcher
