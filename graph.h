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

using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:
  unordered_map<VertexT, map<VertexT, WeightT>> adjList;
  bool _LookupVertex(VertexT v) const{
    if(adjList.find(v) == adjList.end()){
      return false;
    }
    return true;
  }

 public:

  graph(){
    

  }

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
    return static_cast<int>(this->Vertices.size());
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    int count = 0;

    //
    // loop through the adjacency matrix and count how many
    // edges currently exist:
    //
    for (int row = 0; row < this->NumVertices(); ++row) {
      for (int col = 0; col < this->NumVertices(); ++col) {
        if (this->AdjMatrix[row][col].EdgeExists) {
          count++;
        }
      }
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

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT>  S;

    //
    // we need to search the Vertices and find the position
    // of v, that will be the row we need in the adjacency
    // matrix:
    //
    int row = _LookupVertex(v);

    if (row < 0) {  // not found:
      return S;
    }

    //
    // we found the row, so loop along the row and for every
    // edge that exists, add the column vertex to V:
    //
    // NOTE: how many columns are there?  The # of vertices.
    //
    for (int col = 0; col < this->NumVertices(); ++col) {
      if (this->AdjMatrix[row][col].EdgeExists) {
        VertexT dest = this->Vertices[col];  // dest vertex is here:
        S.insert(dest);
      }
    }
    return S;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    return this->Vertices;  // returns a copy:
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
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;
    for (int i = 0; i < this->NumVertices(); ++i) {
      output << " " << i << ". " << this->Vertices[i] << endl;
    }

    output << endl;
    output << "**Edges:" << endl;
    for (int row = 0; row < this->NumVertices(); ++row) {
      output << " row " << row << ": ";

      for (int col = 0; col < this->NumVertices(); ++col) {
        if (this->AdjMatrix[row][col].EdgeExists == false) {
          output << "F ";
        } else {
          output << "(T,"
            << this->AdjMatrix[row][col].Weight
            << ") ";
        }
      }
      output << endl;
    }
    output << "**************************************************" << endl;
  }
};
