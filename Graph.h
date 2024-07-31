#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <vector>

// Define a structure for edges
struct Edge {
    int destination;
    double weight; // Represents travel time or distance
};

// Define a class for the graph
class Graph {
public:
    std::unordered_map<int, std::vector<Edge>> adjacencyList;

    void addEdge(int source, int destination, double weight);
};

#endif // GRAPH_H
