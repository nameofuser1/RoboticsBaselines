#ifndef NODE_HPP
#define NODE_HPP

#include <vector>


struct Node
{
    Node() :
        id(0), x(0), y(0), heuristic_cost(0), occupied(false),
        ctx(nullptr)
    {}

    Node(int id, double x, double y, double cost, bool occupied=false) :
        id(id), x(x), y(y), heuristic_cost(cost), occupied(occupied),
        ctx(nullptr)
    {}

    int id;
    double x, y, heuristic_cost;

    /**
     * @brief occupied  -   flag which indicates whether node contains obstacle
     */
    bool occupied;

    /**
     * @brief edges
     *
     * @member  first - pointer to neighbour node
     * @member  second - cost of the edge
     */
    std::vector<std::pair<Node*, double>> edges;

    /**
     * @brief ctx - context attached to a node.
     */
    void *ctx;
};

#endif // NODE_HPP
