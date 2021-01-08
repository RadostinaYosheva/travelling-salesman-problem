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


void Graph::updatePQ(priorityQ& oldPQ, std::vector<bool> isVisited)
{
    priorityQ newPQ;

    while (!oldPQ.empty())
    {
        int neighbourIndex = oldPQ.top().second.first;

        if (!isVisited[neighbourIndex])
        {
            newPQ.push(oldPQ.top());
        }

        oldPQ.pop();
    }

    oldPQ = newPQ;
}

void Graph::addToPQ(priorityQ& queue, int key, std::vector<bool> isVisited)
{
    for (int i = 0; i < size; i++)
    {
        if (isVisited[i])
        {
            continue;
        }

        int weight = adjacencyMatrix[key][i];
        queue.push({weight, {key, i}});
    }
}

void Graph::findMST()
{
    MSTMatrix.resize(size);
    priorityQ queue;
    std::vector<int> visitedIndices;
    std::vector<bool> isVisited(size, false);
    isVisited[0] = true;
    visitedIndices.push_back(0);


    for (int i = 0; i < size-1; i++) 
    {
        for(int v : visitedIndices)
        {
            addToPQ(queue, v, isVisited);
        }

        int keyCity = queue.top().second.first;
        int nearest = queue.top().second.second;
        int weight = queue.top().first;

        MSTMatrix[keyCity].push_back(nearest);
        MSTMatrix[nearest].push_back(keyCity);
        isVisited[nearest] = true;
        visitedIndices.push_back(nearest);

        updatePQ(queue, isVisited);

    }

    std::cout << "MST Matrix:" << std::endl;
    printMSTMatrix();

}

void Graph::printMSTMatrix()
{
    for (int i = 0; i < size; i++)
    {
        printf("Size of %d is %d\n", i, MSTMatrix[i].size());
    }
}

std::vector<int> Graph::findOddDegreeVertices()
{
    std::vector<int> odds;
    
    for(int i = 0; i < size; i++)
    {
        if (MSTMatrix[i].size() % 2 != 0)
        {
            odds.push_back(i);
        }
    }

    return odds;
}

void Graph::perfectMatching() 
{
    int nearest = INVALID_VALUE;
    int weight;

    // Find nodes with odd degrees in T to get subgraph O
    std::vector<int> odds = findOddDegreeVertices();

    // for each odd node
    while (!odds.empty()) {
        int temp;
        int first = odds[0];
        // std::vector<int>::iterator it;
        weight = MAX;

        for (int i = 1; i < odds.size(); i++) {
            int neighbourIndex = odds[i];
            // if this node is closer than the current closest, update closest and length
            if (adjacencyMatrix[first][neighbourIndex] < weight) {
                weight = adjacencyMatrix[first][neighbourIndex];
                nearest = neighbourIndex;
                temp = i;
            }
        } 
        
        if (nearest != INVALID_VALUE)
        {
            MSTMatrix[first].push_back(nearest);
            MSTMatrix[nearest].push_back(first);
            odds.erase(odds.begin() + temp);
            odds.erase(odds.begin());
        }
    }
}


Graph::Graph(int _size, std::vector<std::vector<int>> _adjacencyMatrix) : size{_size}, adjacencyMatrix{_adjacencyMatrix} {}
