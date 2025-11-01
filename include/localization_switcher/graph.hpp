#pragma once
#include <vector>
#include <string>
#include <functional>

#include "localization_switcher/component/node.hpp"
#include "localization_switcher/component/common_types.hpp"

namespace localization_switcher
{

    class Graph
    {
    public:
        Graph();
        explicit Graph(std::vector<Node> nodes);

        // 値一致によって現在ノードを特定する。見つからなければ nullptr。
        const Node *get_current_node(const SemanticState &snap) const noexcept;

        // 判定関数を渡してノードを決定する場合のオーバーロード
        // 引数などは今後十分に字変わりうるが，その都度strategy側で型の定義をし直す
        const Node *get_current_node(
            const SemanticState &snap,
            NodeSolver solver) const noexcept;

        // グラフ内の全ノードを取得
        const std::vector<Node> &get_all_nodes() const noexcept;

        // idからnodeを返す
        const Node *get_node(std::string node_id) const noexcept;

    private:
        // semantisStateの等価判定 →→これはstruct側が定義すればいい
        // static bool equals_(const SemanticState &input_node, const SemanticState &node_in_graph) noexcept;

        std::vector<Node> nodes_;
    };

} // namespace localization_switcher