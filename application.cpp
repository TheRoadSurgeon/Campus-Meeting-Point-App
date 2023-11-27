// Project 5 application.cpp
// Hristian Tountchev
// 
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Fall 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//
// Application: Runs the all the logic together and calculates the distance
//              based on user input. It takes two inputs from a user that
//              that are buildings in UIC.
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <queue>
#include <stack>

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"


using namespace std;
using namespace tinyxml2;

// Used for the unvisited nodes priority queue
class prioritize
{
public:
  bool operator()(const pair<long long, double>& p1, const pair<long long, double>& p2) const
  {
    return p1.second > p2.second; 
  }
};

const double INF = numeric_limits<double>::max();

/// @brief This function is used to parse string and save a
///        building object. Which will be used for finding distance 
///        between two building objects
/// @param query String based on user input
/// @param Buildings vector for all the building information
/// @return struct with the building a user is searching for
BuildingInfo searchBuilding(string query, const vector<BuildingInfo>& Buildings){
  BuildingInfo bInfo;
  bInfo.Abbrev = "";
  bInfo.Fullname = "";

  // For loop that loops through all the abbreviations
  // a building may have
  for(size_t i = 0; i < Buildings.size(); i++){
    if(Buildings[i].Abbrev == query){
      bInfo = Buildings[i];
      return bInfo;
    }
  }

  // For loop that loops through all the names
  // a building may have
  for(size_t i = 0; i < Buildings.size(); i++){
    if(Buildings[i].Fullname.find(query) != string::npos){
      bInfo = Buildings[i];
      return bInfo;
    }
  }
  
  return bInfo;
}

/// @brief Build the graph structure that will hold all the information o
/// @param Nodes the IDs of vertices and edges 
/// @param G the graph that we are trying to build
/// @param Footways gives the coordinates to calculate the weights of the graph
void buildGraph(const map<long long, Coordinates>&  Nodes, graph<long long, double>& G, const vector<FootwayInfo>& Footways){
  
  for(const auto& elem : Nodes){
    G.addVertex(elem.first);
  }

  
  // For/each loop that goes through all the foot ways.
  // Checks the distances between the two buildings and stores
  // them as an edge in the data structure.
  for(const auto& elem : Footways){
    for(size_t i = 0; i < elem.Nodes.size() - 1; i++){ // maybe -1
      double distance = distBetween2Points(Nodes.at(elem.Nodes.at(i)).Lat, Nodes.at(elem.Nodes.at(i)).Lon, Nodes.at(elem.Nodes.at(i+1)).Lat, Nodes.at(elem.Nodes.at(i+1)).Lon);
      G.addEdge(elem.Nodes.at(i), elem.Nodes.at(i+1), distance);
      G.addEdge(elem.Nodes.at(i+1), elem.Nodes.at(i), distance);
    }
  }
  
}

/// @brief finds the building that is in the middle of two points.
/// @param building1 person 1's building/starting location.
/// @param building2 person 2's building/starting location.
/// @param Buildings all the buildings that are stored and looked at.
/// @return struct type that stores the middle buildings information.
BuildingInfo getMidPoint(const BuildingInfo& building1, const BuildingInfo& building2, const vector<BuildingInfo>& Buildings){
  
  // Stores the coordinates of the middle point of two buildings.
  Coordinates midpoint = centerBetween2Points(building1.Coords.Lat, building1.Coords.Lon, building2.Coords.Lat, building2.Coords.Lon);
  BuildingInfo midBuilding;
  double min = INF;

  // For loop that tries to find the shortest distance between
  // two buildings.
  for(size_t i = 0; i < Buildings.size(); i++){
    double distance = distBetween2Points(midpoint.Lat, midpoint.Lon, Buildings.at(i).Coords.Lat, Buildings.at(i).Coords.Lon);
    
    // Simple min algorithm
    if (distance < min){
      min = distance;
      midBuilding = Buildings.at(i);
    }
  }

  cout << "Destination Building:" << endl;
  cout << " " << midBuilding.Fullname << endl;
  cout << " (" << midBuilding.Coords.Lat << ", " << midBuilding.Coords.Lon << ")"<< endl;
  return midBuilding; // A BuildingInfo struct that holds the midpoint building information.
} 

/// @brief finds the closes destination from a building.
/// @param building builiding we are trying to find the closes location for.
/// @param Footways all the footways that hold all the information for the distances.
/// @param Nodes holds all the information for the nodes.
/// @return coordinates to the closest node of a building.
Coordinates closesNode(const BuildingInfo& building, const vector<FootwayInfo>& Footways, const map<long long, Coordinates>&  Nodes){
  double min = INF;
  Coordinates closeNode;
  for(const auto& elem : Footways){
    
    // for loop that goes through all the nodes.
    // calculates all the distances and finds the closes.
    for(size_t i = 0; i < elem.Nodes.size() - 1; i++){ // maybe its -1
      double distance = distBetween2Points(Nodes.at(elem.Nodes.at(i)).Lat, Nodes.at(elem.Nodes.at(i)).Lon, building.Coords.Lat, building.Coords.Lon);
      if (distance < min){
        min = distance;
        closeNode.Lat = Nodes.at(elem.Nodes.at(i)).Lat;
        closeNode.Lon = Nodes.at(elem.Nodes.at(i)).Lon;
        closeNode.ID = Nodes.at(elem.Nodes.at(i)).ID;
      }
    }
  }
  return closeNode; // Retruns the coordinate struct of the closest node.
}

/// @brief Calculates the shortest distance between two buildings.
/// @param startV the starting location of a persons building.
/// @param G graph structure that holds all the data for all vertices and edges.
/// @param distances map that holds distances IDs and distance. Filled to be used later.
/// @param pred map that holds the predecessor vertices. Filled to be used later.
void DijkstraShortestPath(long long startV, const graph<long long, double>& G, map<long long, double>& distances, map<long long, long long>& pred){

  // Queue of all the vertices that will be visited through the algorithm.
  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> unvisitedQueue;
  set<long long> visitedSet;
  
  // for each loop that fills the distances map with
  // each vertex of the graph and default values (infinite).
  for(const auto& elem : G.getVertices()){
    distances[elem] = INF;
    unvisitedQueue.push({elem, INF});
  }

  // startV has a distance of 0 from itself
  distances[startV] = 0.0;
  unvisitedQueue.push({startV, 0.0});
  

  // Main loop for the algo.
  while (!unvisitedQueue.empty()){
    // Visit vertex with minimum distance from startV
    // Pop off the first vertex in the queue.
    pair<long long, double> currentV = unvisitedQueue.top();
    unvisitedQueue.pop();

    // Check if the value is infinity.
    // If yes we are done.
    if(distances[currentV.first] == INF){
      break;
    }
    // Check if the current vertex is in the set already.
    // If it is skip it.
    else if(visitedSet.count(currentV.first)){
      continue;
    }
    // If the current vertex is not in the set add it.  
    else{
      visitedSet.insert(currentV.first);
    }
    
    // For/each loop checks all the vertices in the graph.
    for(const auto& adj : G.neighbors(currentV.first)){
        double edgeWeight;

        // Get the weight of the current vertex.
        G.getWeight(currentV.first, adj, edgeWeight);
        
        // Used to check for a shorter path.
        double alternativePathDistance = distances[currentV.first] + edgeWeight;

        // If shorter path from startV to adjV is found,
        // update adjV's distance and predecessor
        if(alternativePathDistance < distances[adj]) {
          distances[adj] = alternativePathDistance;
          pred[adj] = currentV.first;
          unvisitedQueue.push({adj, alternativePathDistance});
        }
    }
  } // End of main loop.

}


//
// Implement your standard application here
//
void application(
  map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
  vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
  string person1Building, person2Building;
  map<long long, double> distance1;
  map<long long, double> distance2;
  double totalDist1 = 0.0;
  double totalDist2 = 0.0;
  map<long long, long long> pred1;
  map<long long, long long> pred2;
  stack<long long> stack;
  bool stopLoop = false;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#"){
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);


    BuildingInfo building1 = searchBuilding(person1Building, Buildings);
    BuildingInfo building2 = searchBuilding(person2Building, Buildings);
    
    while(building1.Abbrev == "" || building2.Abbrev == ""){
      if(building1.Abbrev == ""){
        cout << "Person 1's building not found" << endl;
      }
      else if(building2.Abbrev == ""){
        cout << "Person 2's building not found" << endl;
      }
      
      cout << endl;
      cout << "Enter person 1's building (partial name or abbreviation), or #> ";
      getline(cin, person1Building);
      
      if(person1Building == "#"){
        stopLoop = true;
        break;
      }

      cout << "Enter person 2's building (partial name or abbreviation)> ";
      getline(cin, person2Building);
      building1 = searchBuilding(person1Building, Buildings);
      cout << building1.Fullname << endl;
      building2 = searchBuilding(person2Building, Buildings);
      cout << building2.Fullname << endl;
    }

    if(stopLoop){
      break;
    }
    
    cout << endl << "Person 1's point:" << endl;
    cout << " " << building1.Fullname << endl;
    cout << " (" << building1.Coords.Lat << ", " << building1.Coords.Lon << ")" << endl;
    cout << "Person 2's point:" << endl;
    cout << " " << building2.Fullname << endl;
    cout << " (" << building2.Coords.Lat << ", " << building2.Coords.Lon << ")" << endl;
    BuildingInfo midBuilding = getMidPoint(building1, building2, Buildings);
    Coordinates closestToBuilding1 = closesNode(building1, Footways, Nodes);
    Coordinates closestToBuilding2 = closesNode(building2, Footways, Nodes);
    Coordinates closestToMidBuilding = closesNode(midBuilding, Footways, Nodes);

    cout << endl;
    cout << "Nearest P1 node:" << endl;
    cout << " " << closestToBuilding1.ID << endl;
    cout << " (" << closestToBuilding1.Lat << ", " << closestToBuilding1.Lon << ")" << endl;
    cout << "Nearest P2 node:" << endl;
    cout << " " << closestToBuilding2.ID << endl;
    cout << " (" << closestToBuilding2.Lat << ", " << closestToBuilding2.Lon << ")" << endl;
    cout << "Nearest destination node:" << endl;
    cout << " " << closestToMidBuilding.ID << endl;
    cout << " (" << closestToMidBuilding.Lat << ", " << closestToMidBuilding.Lon << ")" << endl;

    DijkstraShortestPath(building1.Coords.ID, G, distance1, pred1);
    DijkstraShortestPath(building2.Coords.ID, G, distance2, pred2);
   
    
    
    // cout << distance1[building2.Coords.ID] << endl;
    for(const auto& elem : distance1){
      if(elem.second < INF){
        cout << "V: " << elem.first << "W: " << elem.second << endl;
        break;
      } 
    }
    // auto it = distance1.begin();
    // while(it != distance1.end()){
    //   long long vertex1 = it->first;
    //   double weight1 = it->second;
    //   double weight;
    //   it++;

    //   if(it != distance1.end()){
    //     double weight2 = it->second;
    //     it++;
    //     G.getWeight(weight1, weight2, weight);
    //   }
      
    //   totalDist1 += weight;
      
    // }
      
    // cout << totalDist1 << endl;
    // if(distance1[building2.Coords.ID] >= INF){
    //   cout << "Sorry, destination unreachable." << endl;
    // }
    // else{
    //   cout << "Person 1's distance to dest: " << distance1[building2.Coords.ID] << endl;
    // }
   

    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }    
}


int main() {
  graph<long long, double> G;

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;


  //
  // TO DO: build the graph, output stats:
  //
  buildGraph(Nodes, G, Footways);

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}
