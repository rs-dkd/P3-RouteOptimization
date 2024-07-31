#include <iostream>
#include "Graph.h"
#include "Algorithms.h"

int main() {
    Graph graph;
    graph.addEdge(0, 1, 10.0);
    graph.addEdge(1, 2, 5.0);
    graph.addEdge(0, 2, 15.0);

    std::vector<double> distances = dijkstra(graph, 0);
    std::cout << "Dijkstra's shortest paths:" << std::endl;
    for (size_t i = 0; i < distances.size(); ++i) {
        std::cout << "Distance from node 0 to node " << i << ": " << distances[i] << std::endl;
    }

    std::vector<int> path = aStar(graph, 0, 2);
    std::cout << "A* path from node 0 to node 2: ";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    return 0;
}
