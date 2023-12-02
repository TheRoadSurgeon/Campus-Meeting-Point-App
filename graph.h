///@file graph.h
///@author Hristian Tountchev
///@date 11/15/2023
/// Adam T Koehler, PhD
/// University of Illinois Chicago
/// CS 251, Fall 2023
///
/// Project Original Variartion By:
/// Joe Hummel, PhD
/// University of Illinois at Chicago
///
///@brief graph.h will handle the organization of our data.
///       It creats a graph out of a map of map.
///       It will handle adding of vertices and the edges between them.

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include "osm.h"


using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:
  map<VertexT, map<VertexT, WeightT>> adjList;
  bool _LookupVertex(VertexT v) const{
    if(adjList.find(v) == adjList.end()){
      return false;
    }
    return true;
  }

 public:

  graph(){}

  /// @brief Shows the number of vertices in the graph
  /// @return the number of vertices
  int NumVertices() const {
    return this->adjList.size();
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    int count = 0;
    
    for(const auto& elem : adjList){
      count+=elem.second.size();
    }
    return count;
  }

  /// @brief This function adds all the vertecies to a map.
  ///        It has a check if a vertex already exists and
  ///        tell the user if the vertex was added or it already exists.
  /// @param v takes in a new vertex to be added to the graph.
  /// @return returns true if the vertex was added to the map.
  bool addVertex(VertexT v) {
    
    // Check if the vertex is in the map already.
    if(_LookupVertex(v)){
      return false;
    }
    // If the key does not exist add it to map.
    adjList[v];

    return true;
  }

  /// @brief This function adds edges to each vertex in the map.
  ///        It handles error check to see if a vertex already exists.
  ///        If the vertex does not exist in the map returns false.
  /// @param from key vertex in the map.
  /// @param to value vertex in the map. Stored as a key of the nested map.
  /// @param weight value in the map. Stored as a value in the nested map.
  /// @return true if vertex is in the map/false if not.
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    
    if(!_LookupVertex(from) || !_LookupVertex(to)){
      return false;
    }

    adjList[from][to] = weight;

    return true;
  }

  /// @brief Returns the weight associated with a given edge.  If
  ///        the edge exists, the weight is returned via the reference
  ///        parameter and true is returned.  If the edge does not
  ///        exist, the weight parameter is unchanged and false is
  ///        returned.
  /// @param from key vertex in the map.
  /// @param to value vertex in the map. Stored as a key of the nested map.
  /// @param weight value in the map. Stored as a value in the nested map.
  /// @return true if vertex is in the map/false if not.
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    
    if(!_LookupVertex(from) || !_LookupVertex(to)){
      return false;
    }
    // Find the vertex in the graph
    auto vertexKey = adjList.find(from);

    // Store the nested map to a temp map (by reference).
    // Allows access of the key/value pair in the nested map
    const map<VertexT, WeightT>& tempMap = vertexKey->second;
    
    // Find the edge that is being looked for.
    auto edgeKey = tempMap.find(to);
    
    // Check if there is an edge in the nested map.
    if(edgeKey != tempMap.end()){
      // Save the weight of the connection (vertex->edge).
      weight = edgeKey->second;
    }
    else{
      return false;
    }
    
    return true;
  }

  
  /// @brief Returns a set containing the neighbors of v, i.e. all
  ///        vertices that can be reached from v along one edge.
  ///        Since a set is returned, the neighbors are returned in
  ///        sorted order; use foreach to iterate through the set.
  /// @param v Vertex we want to find the neighbors for.
  /// @return A set that has all the neighbors of the v vertex.
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT>  S;

    // Make sure the vertex exists in the adjList map.
    if(!_LookupVertex(v)){
      return S;
    }

    auto vertexKey = adjList.find(v);

    const map<VertexT, WeightT>& tempMap = vertexKey->second;

    // For each loop that will save each key (vertex) of the adjList map.
    for(const auto& elem : tempMap){
      S.insert(elem.first);
    }

    return S;
  }


  /// @brief Returns a vector containing all the vertices currently in
  ///        the graph.
  /// @return A vector with all the vertices in the graph
  vector<VertexT> getVertices() const {
    vector<VertexT> V;
    
    for(const auto& elem : adjList){
      V.push_back(elem.first);
    }

    return V;  // Return a vector of all the vertices.
  }



  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
  //   output << "***************************************************" << endl;
  //   output << "********************* GRAPH ***********************" << endl;

  //   output << "**Num vertices: " << this->NumVertices() << endl;
  //   output << "**Num edges: " << this->NumEdges() << endl;

  //   output << endl;
  //   output << "**Vertices:" << endl;
  //   for (int i = 0; i < this->NumVertices(); ++i) {
  //     output << " " << i << ". " << this->Vertices[i] << endl;
  //   }

  //   output << endl;
  //   output << "**Edges:" << endl;
  //   for (int row = 0; row < this->NumVertices(); ++row) {
  //     output << " row " << row << ": ";

  //     for (int col = 0; col < this->NumVertices(); ++col) {
  //       if (this->AdjMatrix[row][col].EdgeExists == false) {
  //         output << "F ";
  //       } else {
  //         output << "(T,"
  //           << this->AdjMatrix[row][col].Weight
  //           << ") ";
  //       }
  //     }
  //     output << endl;
  //   }
  //   output << "**************************************************" << endl;
  }
};
