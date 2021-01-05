#include <iostream>
#include <vector>

class Graph {
    private:
    const int size;
    std::vector<std::vector<int>> adjacencyMatrix;
    // std::stack<int> path; To print the indeces of the path (??)

    int findShortestPath(int startVertex);                              /* (NN) Finding the shortest path from a given city */
    int findMinWeightIndex(int vertex, std::vector<bool> visited);      /* (NN) Find the smallest distance between a given city and his neighbours */
    
    public:
    Graph() = default;
    Graph(int _size, std::vector<std::vector<int>> _adjacencyMatrix);
    void nearestNeighbour();
    // second and third algorithms implementation

};
