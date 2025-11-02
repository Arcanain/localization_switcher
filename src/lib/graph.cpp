// src/graph.cpp
#include "localization_switcher/lib/graph.hpp"

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
            if (snap == n.semantic()) return &n;
        }
        return nullptr;
    }

    const Node *Graph::get_current_node(const SemanticState &snap, Selector solver) const noexcept
    {
        return solver ? solver(snap, nodes_) : get_current_node(snap);
        // return solver(snap, nodes_); // 直接参照を渡す
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
            if (n.id() == node_id) return &n;
        }
        return nullptr;
    }

} // namespace localization_switcher
