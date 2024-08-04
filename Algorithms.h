#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "GraphAdjList.h"
#include "data_src/OSMVertex.h"
#include <vector>
#include <DataSource.h>
#include <data_src/OSMVertex.h>

using namespace bridges;
using namespace bridges::dataset;
using namespace bridges::datastructure;

// Function declarations for algorithms
std::vector<int> dijkstra(const GraphAdjList<int, OSMVertex, double>& graph, int startNode, int goalNode, std::unordered_map<int, double>& distances);
std::vector<int> aStar(const GraphAdjList<int, OSMVertex, double>& graph, int startNode, int goalNode);

#endif // ALGORITHMS_H
