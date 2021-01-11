#ifndef _GRAPH_H
#define _GRAPH_H

#include <iostream>
#include <vector>
#include <queue>


typedef int vertex;
typedef int edgeWeight;
typedef std::pair<edgeWeight, vertex> edgeVertexPair;
typedef std::pair<vertex, vertex>  neighboursPair;
typedef std::pair<edgeWeight, neighboursPair>  edgePair;
typedef std::priority_queue< edgePair, 
                             std::vector<edgePair>, 
                             std::greater<edgePair> >  priorityQ;
typedef std::vector<std::vector<edgeWeight>> matrix;
typedef std::vector<std::vector<vertex>> list;


class Graph {
  public:
    // Functions returning shortest path for algorithm
    int nearestNeighbourAlgorithm();
    int christofidesAlgorithm();
    int aStarAlgorithm();

    Graph(int _size, matrix _adjacencyMatrix);

  private:
    int size;
    matrix adjacencyMatrix;
    list adjListMST;
    
    list findMST(matrix currAdjMatrix, 
                 std::vector<bool> isVisited, 
                 vertex start, int numVisited);                                     /* Returns an adjacency list of MinimumSpanningTree of a graph */
    void addToPQ(priorityQ& queue, vertex key, std::vector<bool> isVisited);        /* Adds all unvisited edges of given vertex to priority queue (Used for MST)*/
    void updatePQ(priorityQ& oldPQ, std::vector<bool> isVisited);                   /* Removes all edges where second vertex is the last visited vertex (Used for MST) */
    int getPathLength(std::vector<vertex> path);
    void setAdjListMST(list adjListMST);

    
    // Functions used in Nearest Neighbour Algorithm
    int findShortestPath(vertex startVertex);                                       /* Finds the shortest path from a given vertex */
    vertex findNextNeighbourIndex(vertex vertex, std::vector<bool> visited);        /* Finds the index of the nearest neighbour */

    
    // Functions used in Christofides' Algorithm
    std::vector<vertex> findOddDegreeVertices();                                    /* Finds all the vertices with odd degree */
    void perfectMatching();                                                         /* Matches all the vertices with odd degree */
    std::vector<vertex> getEulerianPath();                                          /* Finds the path where every edge of the MST is visited */
    void makeHamiltonian(std::vector<vertex> &path);                                /* Takes shortcuts in path (Removes repeating vertices) */
    
    
    // Functions used in A* Algorithm
    int getMSTLength(list MST);
    edgeWeight getCost(std::priority_queue<edgeVertexPair, 
                std::vector<edgeVertexPair>, 
                std::greater<edgeVertexPair> > &queue, vertex index);               /* Removes a vertex and returns its cost */
    void removeFromAdjList(list &adjList, vertex index);                            /* Removes vertex from adjacency list */
};

#endif
