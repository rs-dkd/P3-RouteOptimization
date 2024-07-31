#include "Algorithms.h"
#include <queue>
#include <limits>
#include <unordered_map>
#include <algorithm>

using NodeDistPair = std::pair<double, int>;

// Dijkstra's Algorithm
std::vector<double> dijkstra(const Graph& graph, int startNode) {
    std::unordered_map<int, double> distances;
    for (const auto& pair : graph.adjacencyList) {
        distances[pair.first] = std::numeric_limits<double>::infinity();
    }
    distances[startNode] = 0.0;

    std::priority_queue<NodeDistPair, std::vector<NodeDistPair>, std::greater<>> priorityQueue;
    priorityQueue.push({0.0, startNode});

    while (!priorityQueue.empty()) {
        double currentDistance = priorityQueue.top().first;
        int currentNode = priorityQueue.top().second;
        priorityQueue.pop();

        if (currentDistance > distances[currentNode]) continue;

        for (const auto& edge : graph.adjacencyList.at(currentNode)) {
            double newDistance = currentDistance + edge.weight;
            if (newDistance < distances[edge.destination]) {
                distances[edge.destination] = newDistance;
                priorityQueue.push({newDistance, edge.destination});
            }
        }
    }

    std::vector<double> result;
    for (const auto& pair : distances) {
        result.push_back(pair.second);
    }
    return result;
}

// A* Algorithm
double heuristic(int node1, int node2) {
    return 0.0; // Placeholder
}

std::vector<int> aStar(const Graph& graph, int startNode, int goalNode) {
    std::unordered_map<int, double> gScore, fScore;
    std::unordered_map<int, int> cameFrom;
    for (const auto& pair : graph.adjacencyList) {
        gScore[pair.first] = std::numeric_limits<double>::infinity();
        fScore[pair.first] = std::numeric_limits<double>::infinity();
    }
    gScore[startNode] = 0.0;
    fScore[startNode] = heuristic(startNode, goalNode);

    std::priority_queue<NodeDistPair, std::vector<NodeDistPair>, std::greater<>> openSet;
    openSet.push({fScore[startNode], startNode});

    while (!openSet.empty()) {
        int currentNode = openSet.top().second;
        openSet.pop();

        if (currentNode == goalNode) {
            std::vector<int> path;
            while (cameFrom.find(currentNode) != cameFrom.end()) {
                path.push_back(currentNode);
                currentNode = cameFrom[currentNode];
            }
            path.push_back(startNode);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& edge : graph.adjacencyList.at(currentNode)) {
            double tentativeGScore = gScore[currentNode] + edge.weight;

            if (tentativeGScore < gScore[edge.destination]) {
                cameFrom[edge.destination] = currentNode;
                gScore[edge.destination] = tentativeGScore;
                fScore[edge.destination] = gScore[edge.destination] + heuristic(edge.destination, goalNode);
                openSet.push({fScore[edge.destination], edge.destination});
            }
        }
    }

    return {}; // Return an empty path if there is no solution
}
