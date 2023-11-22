// application.cpp <Starter Code>
// <Your name>
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
// 
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

#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"


using namespace std;
using namespace tinyxml2;


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

  for(size_t i = 0; i < Buildings.size(); i++){
    if(Buildings[i].Abbrev == query){
      bInfo = Buildings[i];
      return bInfo;
    }
  }
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

  
  for(const auto& elem : Footways){

    for(int i = 0; i < elem.Nodes.size() - 1; i++){ // maybe -1
      double distance = distBetween2Points(Nodes.at(elem.Nodes.at(i)).Lat, Nodes.at(elem.Nodes.at(i)).Lon, Nodes.at(elem.Nodes.at(i+1)).Lat, Nodes.at(elem.Nodes.at(i+1)).Lon);
      G.addEdge(elem.Nodes.at(i), elem.Nodes.at(i+1), distance);
      G.addEdge(elem.Nodes.at(i+1), elem.Nodes.at(i), distance);
    }
  }
  
}

BuildingInfo getMidPoint(const BuildingInfo& building1, const BuildingInfo& building2, const vector<BuildingInfo>& Buildings){
  Coordinates midpoint = centerBetween2Points(building1.Coords.Lat, building1.Coords.Lon, building2.Coords.Lat, building2.Coords.Lon);
  BuildingInfo midBuilding;
  double min = INF;
  for(int i = 0; i < Buildings.size(); i++){
    double distance = distBetween2Points(midpoint.Lat, midpoint.Lon, Buildings.at(i).Coords.Lat, Buildings.at(i).Coords.Lon);
    
    if (distance < min){
      min = distance;
      midBuilding = Buildings.at(i);
    }
  }
  cout << "Destination Building:" << endl;
  cout << " " << midBuilding.Fullname << endl;
  cout << " (" << midBuilding.Coords.Lat << ", " << midBuilding.Coords.Lon << ")"<< endl;
  return midBuilding;
} 

Coordinates closesNode(const BuildingInfo& building, const vector<FootwayInfo>& Footways, const map<long long, Coordinates>&  Nodes){
  double min = INF;
  Coordinates closeNode;
  for(const auto& elem : Footways){

    for(int i = 0; i < elem.Nodes.size() - 1; i++){ // maybe its -1
      double distance = distBetween2Points(Nodes.at(elem.Nodes.at(i)).Lat, Nodes.at(elem.Nodes.at(i)).Lon, building.Coords.Lat, building.Coords.Lon);
      if (distance < min){
        min = distance;
        closeNode.Lat = Nodes.at(elem.Nodes.at(i)).Lat;
        closeNode.Lon = Nodes.at(elem.Nodes.at(i)).Lon;
        closeNode.ID = Nodes.at(elem.Nodes.at(i)).ID;
      }
    }
  }
  return closeNode;
}

//
// Implement your standard application here
//
void application(
  map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways,
  vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
  string person1Building, person2Building;
  bool stopLoop = false;

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);


    //
    // TO DO: lookup buildings, find nearest start and dest nodes, find center
    // run Dijkstra's alg from each start, output distances and paths to destination:
    //

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
