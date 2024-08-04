#include "Algorithms.h"
#include "GraphAdjList.h"
#include <queue>
#include <limits>
#include <unordered_map>
#include <algorithm>
#include <cmath>

using namespace bridges;
using namespace bridges::dataset;
using namespace bridges::datastructure;

using NodeDistPair = std::pair<double, int>;

// Dijkstra's Algorithm
std::vector<int> dijkstra(const GraphAdjList<int, OSMVertex, double>& graph, int startNode, int goalNode, std::unordered_map<int, double>& distances) {
    for (const auto& pair : *graph.getVertices()) {
        distances[pair.first] = std::numeric_limits<double>::infinity();
    }
    distances[startNode] = 0.0;

    std::priority_queue<NodeDistPair, std::vector<NodeDistPair>, std::greater<>> priorityQueue;
    priorityQueue.push({ 0.0, startNode });

    std::unordered_map<int, int> cameFrom;

    while (!priorityQueue.empty()) {
        double currentDistance = priorityQueue.top().first;
        int currentNode = priorityQueue.top().second;
        priorityQueue.pop();

        if (currentNode == goalNode) {
            std::vector<int> path;
            while (currentNode != startNode) {
                path.push_back(currentNode);
                currentNode = cameFrom[currentNode];
            }
            path.push_back(startNode);
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (currentDistance > distances[currentNode]) continue;

        for (const auto& edge : graph.outgoingEdgeSetOf(currentNode)) {
            int destNode = edge.to();
            double newDistance = currentDistance + edge.getEdgeData();
            if (newDistance < distances[destNode]) {
                distances[destNode] = newDistance;
                cameFrom[destNode] = currentNode;
                priorityQueue.push({ newDistance, destNode });
            }
        }
    }

    return {}; // Return an empty path if there is no solution
}

// Heuristic function for A*
double heuristic(const GraphAdjList<int, OSMVertex, double>& graph,int node1, int node2) {
    const OSMVertex& v1 = graph.getVertex(node1)->getValue();
    const OSMVertex& v2 = graph.getVertex(node2)->getValue();
    double coords1[2], coords2[2];
    v1.getCartesianCoords(coords1);
    v2.getCartesianCoords(coords2);
    double dx = coords1[0] - coords2[0];
    double dy = coords1[1] - coords2[1];
    return std::sqrt(dx * dx + dy * dy);
}

// A* Algorithm
std::vector<int> aStar(const GraphAdjList<int, OSMVertex, double>& graph, int startNode, int goalNode) {
    std::unordered_map<int, double> gScore, fScore;
    std::unordered_map<int, int> cameFrom;
    for (const auto& pair : *graph.getVertices()) {
        gScore[pair.first] = std::numeric_limits<double>::infinity();
        fScore[pair.first] = std::numeric_limits<double>::infinity();
    }
    gScore[startNode] = 0.0;
    fScore[startNode] = heuristic(graph, startNode, goalNode);

    std::priority_queue<NodeDistPair, std::vector<NodeDistPair>, std::greater<>> openSet;
    openSet.push({ fScore[startNode], startNode });

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

        for (const auto& edge : graph.outgoingEdgeSetOf(currentNode)) {
            int neighbor = edge.to();
            double tentativeGScore = gScore[currentNode] + edge.getEdgeData();

            if (tentativeGScore < gScore[neighbor]) {
                cameFrom[neighbor] = currentNode;
                gScore[neighbor] = tentativeGScore;
                fScore[neighbor] = gScore[neighbor] + heuristic(graph, neighbor, goalNode);
                openSet.push({ fScore[neighbor], neighbor });
            }
        }
    }

    return {}; // Return an empty path if there is no solution
}