#include "graph.h"
#include <limits>

const int INVALID_VALUE = -1;
const int MAX = std::numeric_limits<int>::max();

int Graph::nearestNeighbourAlgorithm() 
{
    int shortestPath = MAX;

    // We can use for loop because we have to visit all cities
    for (int i = 0; i < size; i++) 
    {
        vertex startingPoint = i;
        int currentPath = findShortestPath(startingPoint);

        if (currentPath < shortestPath)
        {
            shortestPath = currentPath;
        }
    }

    return shortestPath;
}


/* Finding the shortest path from a given city */
int Graph::findShortestPath(vertex start) 
{
    int current = start;
    int shortest = 0;
    std::vector<bool> isVisited(size, false);
    isVisited[start] = true;


    for(int i = 0; i < size; i++) 
    {
        vertex nearestNeighbour = findNextNeighbourIndex(current, isVisited);

        if (nearestNeighbour == INVALID_VALUE) {
            shortest += adjacencyMatrix[start][current];
            break;
        }

        shortest += adjacencyMatrix[current][nearestNeighbour];
        current = nearestNeighbour;
        isVisited[nearestNeighbour] = true;
    }

    return shortest;
}


/* Find the index of the nearest neighbour*/
vertex Graph::findNextNeighbourIndex(vertex current, std::vector<bool> isVisited) 
{
    int min = MAX;
    vertex minIndex = INVALID_VALUE;

    for (int i = 0; i < size; i++)
    {
        if (isVisited[i] || i == current)
        {
            continue;
        }

        // It doesn't matter if @vertex is first or second because the matrix is symmetric
        if (min > adjacencyMatrix[current][i])
        {
            min = adjacencyMatrix[current][i];
            minIndex = i;
        }
    }

    return minIndex;
}
