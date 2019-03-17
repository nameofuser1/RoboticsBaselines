#ifndef EDGE_HPP
#define EDGE_HPP

struct Edge
{
    Edge() :
        id_from(0), id_to(0), cost(0), bidir(false)
    {}

    Edge(int id_from, int id_to, double cost, bool bidir) :
        id_from(id_from), id_to(id_to), cost(cost), bidir(bidir)
    {}

    int id_from, id_to;
    double cost;
    bool bidir;
};

#endif // EDGE_HPP
