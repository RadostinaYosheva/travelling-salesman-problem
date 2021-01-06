#include <limits>
#include "graph.h"

const int INVALID_VALUE = -1;
const int MAX = std::numeric_limits<int>::max();



void Graph::nearestNeighbourAlgorithm() {
    int shortestPath = MAX;

    // We can use for loop because we have to visit all cities
    for (int i = 0; i < size; i++) 
    {
        int startingPoint = i;
        int currentPath = findShortestPath(startingPoint);

        if (currentPath < shortestPath)
        {
            shortestPath = currentPath;
        }
    }

    std::cout << "Shortest path is " << shortestPath << " km." << std::endl;
}


/* (NN) Finding the shortest path from a given city */
int Graph::findShortestPath(int startVertex) {
    int currentVertex = startVertex;
    int shortest = 0;
    std::vector<bool> isVisited(size, false);
    isVisited[startVertex] = true;


    for(int i = 0; i < size; i++) 
    {
        int nearestNeighbour = findNextNeighbourIndex(currentVertex, isVisited);

        if (nearestNeighbour == INVALID_VALUE) {
            shortest += adjacencyMatrix[startVertex][currentVertex];
            break;
        }

        shortest += adjacencyMatrix[currentVertex][nearestNeighbour];
        currentVertex = nearestNeighbour;
        isVisited[nearestNeighbour] = true;
    }

    return shortest;
}


/* (NN) Find the index of the nearest neighbour*/
int Graph::findNextNeighbourIndex(int vertex, std::vector<bool> isVisited) 
{
    int min = MAX;
    int minIndex = INVALID_VALUE;

    for (int i = 0; i < size; i++)
    {
        if (isVisited[i] || i == vertex)
        {
            continue;
        }

        // It doesn't matter if @vertex is first or second because the matrix is symmetric
        if (min > adjacencyMatrix[vertex][i])
        {
            min = adjacencyMatrix[vertex][i];
            minIndex = i;
        }
    }

    return minIndex;
}


Graph::Graph() {}

Graph::Graph(int _size, std::vector<std::vector<int>> _adjacencyMatrix) : size{_size}, adjacencyMatrix{_adjacencyMatrix} {}

Graph& Graph::operator=(const Graph& other) {
    if (this != &other) {
        copyGraph(other);
    }

    return *this;
}

void Graph::copyGraph(const Graph& other) {
    size = other.size;
    adjacencyMatrix = other.adjacencyMatrix;
}
