#ifndef _GRAPH_H
#define _GRAPH_H

#include <iostream>
#include <vector>

class Graph {
    private:
    int size;
    std::vector<std::vector<int>> adjacencyMatrix;
    // TODO: std::stack<int> path; To print the indeces of the path (??)

    void copyGraph(const Graph& other);
    int findShortestPath(int startVertex);                              /* (NN) Finding the shortest path from a given city */
    int findMinWeightIndex(int vertex, std::vector<bool> visited);      /* (NN) Find the smallest distance between a given city and his neighbours */
    
    public:
    Graph();
    Graph(int _size, std::vector<std::vector<int>> _adjacencyMatrix);
    Graph& operator= (const Graph& other);
    void nearestNeighbour();
    // TODO: second and third algorithms implementation

};

#endif
