#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <assert.h>
#include <vector>
#include <functional>
#include "edge.hpp"
#include "node.hpp"


namespace AStar
{

namespace internal
{

static const double COST_INF = std::numeric_limits<double>::max();

/**
 * @brief AStar node context attached to each node
 */
struct node_context
{
    node_context() :
        total_cost(COST_INF), total_edge_cost(COST_INF),
        visited(false), in_open(false),
        parent(nullptr)
    {}

    double total_cost;
    double total_edge_cost;
    bool visited;
    bool in_open;

    Node *parent;
};

}


/**
 * @brief Function template to find path between two vertices using A*
 * @note  Since it dynamically allocate memory for vertices context, it is
 *          good to make use of custom allocators. Therefore this function is compatible
 *          with std allocators.
 *
 * @param [in]  start_node      -   node to find path from
 * @param [out] goal_node       -   node to find path to
 * @param [in]  heur_cost_fcn   -   function which calculates heuristic cost between two vertices
 * @param [out] path            -   vector of Nodes from start to goal
 *
 * @return  True/False when path found/not found correspondinly.
 */
template <template <class T> class Allocator = std::allocator>
bool plan_path(Node *start_node, Node *goal_node,
               std::function<double (Node *, Node *)> &heur_cost_fcn,
               std::vector<Node*> &path)
{
    /**
     * Create memory allocation/deallocation lambdas on top of passed allocator type
     */
    Allocator<internal::node_context> context_allocator;
    auto alloc_context = [&context_allocator]() -> internal::node_context* {
        internal::node_context *ctx = context_allocator.allocate(1);
        context_allocator.construct(ctx);
        return ctx;
    };

    auto dealloc_context = [&context_allocator](internal::node_context *ctx) {
        context_allocator.destroy(ctx);
        context_allocator.deallocate(ctx, 1);
    };


    /**
     * Use heap in order to track the lowest cost node
     */
    std::vector<Node*> open_heap;
    std::vector<Node*> visited;
    open_heap.push_back(start_node);

    /**
     * Create context for start node
     */
    internal::node_context *start_node_ctx = alloc_context();
    start_node_ctx->total_cost = heur_cost_fcn(start_node, goal_node);
    start_node_ctx->total_edge_cost = 0.0;
    start_node->ctx = start_node_ctx;

    /**
     * Function which is used to compare items when pushing to/popping from open heap
     */
    auto heap_cmp = [] (Node *node1, Node *node2) -> bool {
        assert(node1->ctx != nullptr);
        assert(node2->ctx != nullptr);

        internal::node_context *ctx1 = static_cast<internal::node_context*>(node1->ctx);
        internal::node_context *ctx2 = static_cast<internal::node_context*>(node2->ctx);

        return ((ctx1->total_cost) > (ctx2->total_cost));
    };

    while(open_heap.size() > 0)
    {
        Node *current_node = open_heap.front();
        std::pop_heap(open_heap.begin(), open_heap.end(), heap_cmp);
        open_heap.pop_back();

        internal::node_context *ctx = static_cast<internal::node_context*>(current_node->ctx);
        ctx->visited = true;
        visited.push_back(current_node);

        double current_total_edge_cost = ctx->total_edge_cost;

        if(current_node == goal_node)
        {
            break;
        }

        /**
         *  Iterate over all the current node's edges
         */
        for(std::pair<Node*, double> &edge: current_node->edges)
        {
            Node *neighbour = edge.first;
            if(neighbour->occupied)
            {
                continue;
            }

            internal::node_context *neighbour_ctx;
            if(neighbour->ctx == nullptr)
            {
                neighbour_ctx = alloc_context();
                neighbour->ctx = neighbour_ctx;
            }
            else
            {
                neighbour_ctx = static_cast<internal::node_context*>(neighbour->ctx);
            }

            if(!neighbour_ctx->visited)
            {
                double edge_cost = edge.second;
                double new_neighbour_edge_total_cost = current_total_edge_cost + edge_cost;
                double new_neighbour_total_cost = new_neighbour_edge_total_cost + heur_cost_fcn(neighbour,
                                                                                                goal_node);
                if(new_neighbour_total_cost < neighbour_ctx->total_cost)
                {
                    neighbour_ctx->total_edge_cost = new_neighbour_edge_total_cost;
                    neighbour_ctx->total_cost = new_neighbour_total_cost;
                    neighbour_ctx->parent = current_node;
                }

                if(!neighbour_ctx->in_open)
                {
                    neighbour_ctx->in_open = true;
                    open_heap.push_back(neighbour);
                    std::push_heap(open_heap.begin(), open_heap.end(), heap_cmp);
                }
            }
        }
    }

    bool result = false;
    if(goal_node->ctx != nullptr)
    {
        result = static_cast<internal::node_context*>(goal_node->ctx)->visited;
    }

    if(result)
    {
        Node *path_node = goal_node;
        while(path_node != nullptr)
        {
            internal::node_context *ctx = static_cast<internal::node_context*>(path_node->ctx);

            path.push_back(path_node);
            path_node = ctx->parent;
        }

        std::reverse(path.begin(), path.end());
    }

    /**
     * Clear context for each visited node
     */
    for(Node *node : visited)
    {
        dealloc_context(static_cast<internal::node_context*>(node->ctx));
    }

    /**
     * Clear context for each node in open list
     */
    for(Node *node : open_heap)
    {
        dealloc_context(static_cast<internal::node_context*>(node->ctx));
    }

    return result;
}

}
#endif // ASTAR_HPP
