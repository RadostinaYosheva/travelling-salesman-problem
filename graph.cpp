#include <limits>
#include "graph.h"

const int INVALID_VALUE = -1;
const int MAX = std::numeric_limits<int>::max();

Graph::Graph(int _size, std::vector<std::vector<int>> _adjacencyMatrix) : size{_size}, adjacencyMatrix{_adjacencyMatrix} {}

void Graph::nearestNeighbour() {
    int shortestPath = MAX;

    // We can use for loop because we have to visit all cities
    for (int i = 0; i < size; i++) 
    {
        int startingPoint = i;
        int currentPath = findShortestPath(startingPoint);

        shortestPath = std::min(findShortestPath(startingPoint), shortestPath);
    }

    std::cout << "Shortest path is " << shortestPath << " km." << std::endl;
    //  Print the path
}

int Graph::findShortestPath(int startVertex) {
    int currentVertex = startVertex;
    int shortest = 0;
    std::vector<bool> isNotVisited(size, true);
    isNotVisited[startVertex] = false;


    for(int i = 0; i < size; i++) 
    {
        int nearestNeighbour = findMinWeightIndex(currentVertex, isNotVisited);

        if (nearestNeighbour == INVALID_VALUE) {
            shortest += adjacencyMatrix[startVertex][currentVertex];
            break;
        }

        shortest += adjacencyMatrix[currentVertex][nearestNeighbour];
        currentVertex = nearestNeighbour;
        isNotVisited[nearestNeighbour] = false;
    }

    return shortest;
}

int Graph::findMinWeightIndex(int vertex, std::vector<bool> isNotVisited) 
{
    int min = MAX;
    int minIndex = INVALID_VALUE;

    for (int i = 0; i < size; i++)
    {
        if (!isNotVisited[i])
        {
            continue;
        }

        // It doesn't matter if @vertex is first or second because the matrix is symmetric
        if (min > adjacencyMatrix[vertex][i] && adjacencyMatrix[vertex][i] != 0)
        {
            min = adjacencyMatrix[vertex][i];
            minIndex = i;
        }
    }

    return minIndex;
}

