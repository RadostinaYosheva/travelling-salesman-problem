#include <algorithm>
#include <stack>
#include "graph.h"


/* Function that returns a MinimumSpanningTree of a graph */
list Graph::findMST(matrix adjMatrix, std::vector<bool> isVisited, vertex start, int numVisited)
{
    int matrixSize = adjMatrix.size();
    list result(matrixSize);
    priorityQ queue;
    std::vector<int> visitedIndices;

    isVisited[start] = true;
    visitedIndices.push_back(start);


    // We are searching for MST for the unvisited vertices
    for (int i = 0; i < matrixSize-numVisited; i++) 
    {
        for(vertex v : visitedIndices)
        {
            // Adding v's edges to priority queue
            addToPQ(queue, v, isVisited);
        }

        vertex keyVertex = queue.top().second.first;
        vertex nearest = queue.top().second.second;
        edgeWeight weight = queue.top().first;

        // add shortest edge to MST
        result[keyVertex].push_back(nearest);
        result[nearest].push_back(keyVertex);
        // mark the next vertex as visited (keyCity is always visited)
        isVisited[nearest] = true;
        // add next city to path
        visitedIndices.push_back(nearest);

        // remove all edges where both ends are visited
        updatePQ(queue, isVisited);
    }

    return result;
}


/* Adds all unvisited edges of given vertex to priority queue */
void Graph::addToPQ(priorityQ& queue, vertex key, std::vector<bool> isVisited)
{
    for (int i = 0; i < size; i++)
    {
        if (isVisited[i])
        {
            continue;
        }

        edgeWeight weight = adjacencyMatrix[key][i];
        queue.push({weight, {key, i}});
    }
}


/* Removes all edges where second vertex is the last visited vertex */
void Graph::updatePQ(priorityQ& oldPQ, std::vector<bool> isVisited)
{
    priorityQ newPQ;

    while (!oldPQ.empty())
    {
        vertex neighbourIndex = oldPQ.top().second.first;

        if (!isVisited[neighbourIndex])
        {
            newPQ.push(oldPQ.top());
        }

        oldPQ.pop();
    }

    oldPQ = newPQ;
}


int Graph::getPathLength(std::vector<vertex> path)
{
    int length = 0;
    vertex start = path[0];
    vertex first, second;

    for(int i = 0; i < path.size() - 1; i++)
    {
        first = path[i];
        second = path[i+1];
        length += adjacencyMatrix[first][second];
    }

    length += adjacencyMatrix[second][start];

    return length;
}


void Graph::setAdjListMST(list listMST)
{
    adjListMST.resize(listMST.size());
    for (int i = 0; i < listMST.size(); i++)
    {
        for (int j = 0; j < listMST[i].size(); j++)
        {
            adjListMST[i].push_back(listMST[i][j]);
        }
    }
}


Graph::Graph(int _size, matrix _adjacencyMatrix) : size{_size}, adjacencyMatrix{_adjacencyMatrix} {}
