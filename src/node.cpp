#include "localization_switcher/node.hpp"

namespace localization_switcher
{


Node::Node(const std::string & id)
    : id_(id)
    {}

    bool Node::operator==(const Node& other) const{
        return id_ == other.id_;
    }

    bool Node::operator!=(const Node& other) const{
        return id_ != other.id_;
    }

    std::string Node::to_string()const{
    return id_;
    }

    void Node::add_next(Node* next_node){
        if(std::find(next_nodes_.begin(), next_nodes_.end(),next_node) == next_nodes_.end()){
            next_nodes_.push_back(next_node);
        }
    }

    const std::vector<Node*>& Node::get_next() const{
        return next_nodes_;
    }

} // namespace localization_switcher
