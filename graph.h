#ifndef _GRAPH_H
#define _GRAPH_H

#include <iostream>
#include <vector>
#include <queue>


typedef int vertex;
typedef int edgeWeight;
// FIXME: edgeVertexPair
typedef std::pair<edgeWeight, vertex> evPair;
typedef std::pair<vertex, vertex>  neighboursPair;
typedef std::pair<edgeWeight, neighboursPair>  edgePair;
typedef std::priority_queue< edgePair, 
                             std::vector<edgePair>, 
                             std::greater<edgePair> >  priorityQ;
typedef std::vector<std::vector<edgeWeight>> matrix;
typedef std::vector<std::vector<vertex>> list;


class Graph {
  public:
  // FIXME: make algorithms namespace -> friend functions
    // Functions returning shortest path for algorithm
    int nearestNeighbourAlgorithm();
    int christofidesAlgorithm();
    int aStarAlgorithm();
    Graph(int _size, matrix _adjacencyMatrix);

  private:
    int size;
    matrix adjacencyMatrix;
    list adjListMST;
    
    void setAdjListMST(list adjListMST);
    list findMST(matrix currAdjMatrix, 
                 std::vector<bool> isVisited, 
                 vertex start, int numVisited);                                 /* Function that returns a MinimumSpanningTree of a graph */
    int getPathLength(std::vector<int> path);

    // Functions used in Nearest Neighbour Algorithm
    int findShortestPath(int startVertex);                                      /* Finds the shortest path from a given vertex */
    int findNextNeighbourIndex(int vertex, std::vector<bool> visited);          /* Finds the index of the nearest neighbour */

    // Functions used in Christofides' Algorithm
    void addToPQ(priorityQ& queue, int key, std::vector<bool> isVisited);       /* Adds all unvisited edges of given vertex to priority queue */
    void updatePQ(priorityQ& oldPQ, std::vector<bool> isVisited);               /* Removes all edges where second vertex is the last visited vertex */
    std::vector<int> findOddDegreeVertices();                                   /* Finds all the vertices with odd degree */
    void perfectMatching();                                                     /* Matches all the vertices with odd degree */
    std::vector<int> getEulerianPath();                                         /* Finds the path where every edge of the MST is visited */
    void makeHamiltonian(std::vector<int> &path);                               /* Takes shortcuts in path (Removes repeating vertices) */
    
    // Functions used in A* Algorithm
    int getMSTLength(list MST);
    void removeFromAdjList(list &adjList, vertex index);                        /* Removes vertex from adjacency list */
    int getCost(std::priority_queue<evPair, 
                std::vector<evPair>, 
                std::greater<evPair> > &queue, vertex index);                   /* Removes a vertex and returns its cost */
};

#endif
