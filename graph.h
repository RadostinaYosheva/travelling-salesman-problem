#ifndef _GRAPH_H
#define _GRAPH_H

#include <iostream>
#include <vector>
#include <queue>


typedef std::pair<int, int>  neighboursPair;
typedef std::pair<int, neighboursPair>  edgePair;
typedef std::priority_queue< edgePair, 
                             std::vector<edgePair>, 
                             std::greater<edgePair> >  priorityQ;
typedef std::vector<std::vector<int>> matrix;
typedef std::vector<std::vector<int>> list;


class Graph {
  public:
    void nearestNeighbourAlgorithm();
    void christofidesAlgorithm();
    Graph(int _size, matrix _adjacencyMatrix);
    
  private:
    int size;
    matrix adjacencyMatrix;
    list adjListMST;

    int findShortestPath(int startVertex);                                      /* (NN) Finding the shortest path from a given city */
    int findNextNeighbourIndex(int vertex, std::vector<bool> visited);          /* (NN) Find the index of the nearest neighbour*/
    void setAdjListMST(list adjListMST);
    list findMST(matrix currAdjMatrix);
    void addToPQ(priorityQ& queue, int key, std::vector<bool> isVisited); 
    void updatePQ(priorityQ& oldPQ, std::vector<bool> isVisited);
    void printMSTMatrix();
    std::vector<int> findOddDegreeVertices();
    void perfectMatching();
    void printPath(std::vector<int> path);
    void makeHamiltonian(std::vector<int> &path);                               // Take shortcuts in path
    std::vector<int> getEulerianPath();
    int getPathLength(std::vector<int> path);
};

#endif
