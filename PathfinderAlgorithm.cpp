#include "Bridges.h"
#include "GraphAdjList.h"
#include "Algorithms.h"
#include "data_src/OSMData.h"
#include <unordered_map>
#include <vector>
#include <iostream>
#include <limits>
#include <queue>
#include <chrono>

using namespace std;
using namespace bridges;
using namespace bridges::dataset;
using namespace bridges::datastructure;

//Finds the center of the map by taking the average of the latitude and longitude
pair<double, double> mapCenter(GraphAdjList<int, OSMVertex, double>& graph){
    double totalLat = 0.0;
    double totalLong = 0.0;
    int vertexCount = graph.getVertices()->size();

    for(const auto& pair : *graph.getVertices()){
        const OSMVertex& vertData = graph.getVertexData(pair.first);
        totalLat += vertData.getLatitude();
        totalLong += vertData.getLongitude();
    }

    return{totalLat / vertexCount, totalLong / vertexCount};
}

//Used to find the vertex closest to a location given by a set of coordinates
int findClosestVertex(GraphAdjList<int, OSMVertex, double>& graph, pair<double, double> coordinates){
    int closestVertex = -1;
    double minDistance = numeric_limits<double>::infinity();

    //Loops through vertices and calculates distance to find the nearest vertex
    for(const auto& pair : *graph.getVertices()){
        const OSMVertex& vertData = graph.getVertexData(pair.first);
        double distance = sqrt(pow(vertData.getLatitude() - coordinates.first, 2) + pow(vertData.getLongitude() - coordinates.second, 2));
        if(distance < minDistance){
            minDistance = distance;
            closestVertex = pair.first;
        }
    }

    return closestVertex;
}

//Uses bfs to check if there is a path to go from start node to goal node
bool canReach(GraphAdjList<int, OSMVertex, double>& graph, int startNode, int goalNode){
    unordered_set<int> visited;
    queue<int> q;
    q.push(startNode);
    visited.insert(startNode);

    while(!q.empty()){
        int currentNode = q.front();
        q.pop();

        if(currentNode == goalNode){
            return true;
        }

        for(const auto& edge : graph.outgoingEdgeSetOf(currentNode)){
            int neighbor = edge.to();
            if(visited.find(neighbor) == visited.end()){
                visited.insert(neighbor);
                q.push(neighbor);
            }
        }
    }

    return false;
}

//Sets up the visualization of Dijkstra's Algorithm
void visualizeDijkstra(GraphAdjList<int, OSMVertex, double>& graph, const unordered_map<int, double>& distances, const vector<int>& path, Bridges& bridges){
    //Colors node based on distance, those that Dijkstra's goes through get colored yellow,
    //and the ones that it doesn't reach are colored grey
    for(const auto& pair : distances){
        int node = pair.first;
        double distance = pair.second;
        Color nodeColor;
        if(distance == numeric_limits<double>::infinity()){
            nodeColor = Color(128, 128, 128, 255);
        }
        else{
            nodeColor = Color(255, 255, 0, 255);
        }
        graph.getVisualizer(node)->setColor(nodeColor);
    }

    //Colors optimal path in green and attempts to make it more noticeable
    for(size_t i = 0; i < path.size(); i++){
        int node = path[i];
        graph.getVisualizer(node)->setColor(Color(0, 255, 0, 255));
        if(i > 0){
            int prevNode = path[i - 1];
            if(graph.getLinkVisualizer(prevNode, node)){
                graph.getLinkVisualizer(prevNode, node)->setColor(Color(0, 255, 0, 255));
                graph.getLinkVisualizer(prevNode, node)->setThickness(3.0f);
            }
        }
    }

    bridges.setDataStructure(&graph);
    bridges.visualize();
}

//Visualizes the A-star algorithm
void visualizeAStar(GraphAdjList<int, OSMVertex, double>& graph, const vector<int>& aStarPath, Bridges& bridges){
    //Colors all vertices grey to start with
    for(const auto& pair : *graph.getVertices()){
        graph.getVisualizer(pair.first)->setColor(Color(128, 128, 128, 255));
    }

    //Colors the optimal path in green and attempts to make it more noticeable
    for(size_t i = 0; i < aStarPath.size(); i++){
        int node = aStarPath[i];
        graph.getVisualizer(node)->setColor(Color(0, 255, 0, 255));
        if(i > 0){
            int prevNode = aStarPath[i - 1];
            graph.getLinkVisualizer(prevNode, node)->setColor(Color(0, 255, 0, 255));
            graph.getLinkVisualizer(prevNode, node)->setThickness(3.0f);
        }
    }

    bridges.setDataStructure(&graph);
    bridges.visualize();
}

//Gives prompt for that takes in latitude and longitude values and determines if it is within range
bool promptCoords(double minLat, double maxLat, double minLon, double maxLon, double& lat, double& lon){
    cout << "Enter latitude (" << minLat << " to " << maxLat << "): ";
    cin >> lat;
    if(lat < minLat || lat > maxLat){
        cerr << "Latitude out of range." << endl;
        return false;
    }

    cout << "Enter longitude (" << minLon << " to " << maxLon << "): ";
    cin >> lon;
    if(lon < minLon || lon > maxLon){
        cerr << "Longitude out of range." << endl;
        return false;
    }

    return true;
}

int main(int argc, char** argv){

    //Cities that are available within OSM, more available but limited for user sake
    vector<string> cities = {
        "New York, New York",
        "Los Angeles, California",
        "Chicago, Illinois",
        "Houston, Texas",
        "Orlando, Florida",
        "Gainesville, Florida",
        "Miami Beach, Florida",
        "San Jose, California",
        "Austin, Texas",
        "San Francisco, California"
    };

    //Display city options, take in user input then check for out of range values
    cout << "Select a city from the list below by entering a number:\n";
    for(int i = 0; i < cities.size(); i++){
        cout << i + 1 << ". " << cities[i] << endl;
    }

    int choice;
    cout << "Enter the number for the city you want to visualize: ";
    cin >> choice;


    if(choice < 1 || choice > static_cast<int>(cities.size())){
        cerr << "Invalid choice.\n";
        return -1;
    }

    string selectedCity = cities[choice - 1];
    cout << "Selected: " << selectedCity << endl;

    //Initializes Bridges API andd then gets OSM data for the city
    //Uncomment next line and replace userID and APIKey
    //Bridges bridges(2, "userID", "APIKey");
    DataSource ds(&bridges);
    OSMData osm_data = ds.getOSMData(selectedCity);

    //Finds long and lat ranges for the city to be sent to promptCoords to check user input values
    double latRange[2], lonRange[2];
    osm_data.getLatLongRange(latRange, lonRange);
    double minLat = latRange[0], maxLat = latRange[1];
    double minLon = lonRange[0], maxLon = lonRange[1];

    cout << "Latitude range: " << minLat << " to " << maxLat << endl;
    cout << "Longitude range: " << minLon << " to " << maxLon << endl;

    double startLat, startLon, goalLat, goalLon;

    cout << "Enter start location coordinates:" << endl;
    while(!promptCoords(minLat, maxLat, minLon, maxLon, startLat, startLon)){
        cout << "Please enter valid coordinates." << endl;
    }


    cout << "Enter goal location coordinates:" << endl;
    while(!promptCoords(minLat, maxLat, minLon, maxLon, goalLat, goalLon)){
        cout << "Please enter valid coordinates." << endl;
    }

    //Load up graph using built in Adjacency List graph implementation
    //Set up for proper visualization
    GraphAdjList<int, OSMVertex, double> graph;
    osm_data.getGraph(&graph);
    graph.forceLargeVisualization(true);

    //Use user inputted values as starting and goal nodes and check to see if goal node is reachable
    int startNode = findClosestVertex(graph, {startLat, startLon});
    int goalNode = findClosestVertex(graph, {goalLat, goalLon});

    if(!canReach(graph, startNode, goalNode)){
        cerr << "Goal node is not reachable from the start node." << endl;
        return -1;
    }

    //Run Dijkstra's algorithm then time and visualize result
    unordered_map<int, double> distances;
    auto start = chrono::high_resolution_clock::now();
    vector<int> dijkstraPath = dijkstra(graph, startNode, goalNode, distances);
    visualizeDijkstra(graph, distances, dijkstraPath, bridges);
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> dijkstraDuration = end - start;
    cout << "Dijkstra's Algorithm took " << dijkstraDuration.count() << " seconds." << endl;
    
    //Run A-star algorithm then time and visualize result
    start = chrono::high_resolution_clock::now();
    vector<int> aStarPath = aStar(graph, startNode, goalNode);
    visualizeAStar(graph, aStarPath, bridges);
    end = chrono::high_resolution_clock::now();
    chrono::duration<double> aStarDuration = end - start;
    cout << "A* Algorithm took " << aStarDuration.count() << " seconds." << endl;

    return 0;
}