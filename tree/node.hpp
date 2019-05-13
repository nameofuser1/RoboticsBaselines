#ifndef NODE_HPP
#define NODE_HPP

#include <vector>

namespace Tree
{

template <class DataType, class CostType=double>
struct Node
{
    Node() :
        ctx()
    {}

    Node(const DataType &data) :
        data(data)
    {}

    Node(const DataType &&data) :
        data(data)
    {}



    /**
     * @brief edges
     *
     * @member  first - pointer to neighbour node
     * @member  second - cost of the edge
     */
    std::vector<std::pair<Node*, CostType>> edges;

    /**
     * @brief data  - node specific data
     */
    DataType data;

    /**
     * @brief ctx   -   additional dynamic context which can be attached to a node
     */
    void *ctx;
};

}

#endif // NODE_HPP
