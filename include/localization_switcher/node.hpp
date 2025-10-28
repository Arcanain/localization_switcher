// include/loacalization_switcher/node.hpp
#pragma once
#include <string>
#include <string>
#include <vector>

namespace localization_switcher
{

    // 状態ノード：一意なID(文字列)だけを持つ
    class Node
    {
    public:
        // コンストラクタ：常に明示的にIDを渡して生成する
        explicit Node(const std::string &id) : id_(id) {}

        // 等価性：id が同じなら同一ノードとみなす
        bool operator==(const Node &other) const; 
        bool operator!=(const Node &other) const;

        std::string to_string() const { return id_; }

        void add_next(Node* next_node);
        const std::vector<Node*>& get_next() const;

        std::string id_;
        std::vector<Node*> next_nodes_;
    };

} // namespace localization_switcher
