#include "Graph.h"

// Implement addEdge method for Graph
void Graph::addEdge(int source, int destination, double weight) {
    adjacencyList[source].push_back({destination, weight});
}
