#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "Graph.h"
#include <vector>

// Function declarations for algorithms
std::vector<double> dijkstra(const Graph& graph, int startNode);
std::vector<int> aStar(const Graph& graph, int startNode, int goalNode);

#endif // ALGORITHMS_H
