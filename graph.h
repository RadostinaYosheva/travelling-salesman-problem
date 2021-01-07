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


class Graph {
  public:
    void nearestNeighbourAlgorithm();
    // TODO: second and third algorithms implementation
    void findMST();

    Graph(int _size, std::vector<std::vector<int>> _adjacencyMatrix);
    
  private:
    int size;
    std::vector<std::vector<int>> adjacencyMatrix;
    std::vector<std::vector<int>> MSTMatrix;

    int findShortestPath(int startVertex);                              /* (NN) Finding the shortest path from a given city */
    int findNextNeighbourIndex(int vertex, std::vector<bool> visited);      /* (NN) Find the index of the nearest neighbour*/
    void addToPQ(priorityQ& queue, int key, std::vector<bool> isVisited); 
    void updatePQ(priorityQ& oldPQ, std::vector<bool> isVisited);
    void printMSTMatrix();

};

#endif
