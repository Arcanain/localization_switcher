// src/graph.cpp

#include "localization_switcher/graph.hpp"
// #include "localization_swithcer/strategy.hpp" //ここに，オーバーロードした場合の関数を入れる？

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace localization_switcher
{

    Graph::Graph() = default;

    Graph::Graph(std::vector<Node> nodes)
        : nodes_(std::move(nodes)) {}

    const Node *Graph::get_current_node(const SemanticState &snap) const noexcept
    {
        for (const auto &n : nodes_)
        {
            if (equals_(snap, n.semantic()))
            {
                return &n;
            }
        }
        return nullptr;
    }

    // オーバーロードしたget_current_node
    // graph.cpp
    const Node *Graph::get_current_node(const SemanticState &snap, NodeSolver solver) const noexcept
    {
        return solver(snap, nodes_); // 直接参照を渡す
    }

    const std::vector<Node> &Graph::get_all_nodes() const noexcept
    {
        return nodes_;
    }

    // idに一致するnodeへの参照を返す
    const Node *Graph::get_node(std::string node_id) const noexcept
    {
        for (const auto &n : nodes_)
        {
            if (n.id() == node_id)
            {
                return &n;
            }
        }
        return nullptr;
    }

} // namespace localization_switcher
