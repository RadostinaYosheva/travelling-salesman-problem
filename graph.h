#ifndef _GRAPH_H
#define _GRAPH_H

#include <iostream>
#include <vector>

class Graph {
  public:
    void nearestNeighbourAlgorithm();
    // TODO: second and third algorithms implementation
    // FIXME: delete
    Graph(int _size, std::vector<std::vector<int>> _adjacencyMatrix);
    Graph();
    // FIXME: delete
    Graph& operator= (const Graph& other);
    
  private:
    int size;
    std::vector<std::vector<int>> adjacencyMatrix;

    // FIXME: delete
    void copyGraph(const Graph& other);
    int findShortestPath(int startVertex);                              /* (NN) Finding the shortest path from a given city */
    int findNextNeighbourIndex(int vertex, std::vector<bool> visited);      /* (NN) Find the index of the nearest neighbour*/

};

#endif
