#include <iostream>
#include <memory>

class Node;

class Edge {
public:
    Edge() { std::cout << "Edge Constructed.\n"; }
    ~Edge() { std::cout << "Edge Destroyed.\n"; }

    void setNode(std::shared_ptr<Node>& node) { node_ = node; }

private:
    std::shared_ptr<Node> node_;
};

class Node {
public:
    Node() { std::cout << "Node Constructed.\n"; }
    ~Node() { std::cout << "Edge Destroyed.\n"; }

    void setEdge(std::shared_ptr<Edge>& edge) { edge_ = edge; }

private:
    std::shared_ptr<Edge> edge_;
};

int main() {
    // shared_ptr can be used to shared management for resource
    std::cout << "-------------\n";
    {
        std::shared_ptr<Edge> edge = std::make_shared<Edge>();
        std::shared_ptr<Node> node = std::make_shared<Node>();

        edge->setNode(node);
        node->setEdge(edge);
    }
    std::cout << "-------------\n";

    return 0;
}
