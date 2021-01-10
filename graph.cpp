#include <limits>
#include <algorithm>
#include <stack>
#include "graph.h"

const int INVALID_VALUE = -1;
const int MAX = std::numeric_limits<int>::max();



void Graph::nearestNeighbourAlgorithm() 
{
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
int Graph::findShortestPath(int startVertex) 
{
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

list Graph::findMST(matrix adjMatrix, std::vector<bool> isVisited, vertex start)
{
    int matrixSize = adjMatrix.size();
    list result(matrixSize);
    priorityQ queue;
    std::vector<int> visitedIndices;

    isVisited[start] = true;
    visitedIndices.push_back(start);


    for (int i = 0; i < matrixSize-1; i++) 
    {
        for(int v : visitedIndices)
        {
            addToPQ(queue, v, isVisited);
        }

        int keyCity = queue.top().second.first;
        int nearest = queue.top().second.second;
        int weight = queue.top().first;

        result[keyCity].push_back(nearest);
        result[nearest].push_back(keyCity);
        isVisited[nearest] = true;
        visitedIndices.push_back(nearest);

        updatePQ(queue, isVisited);
    }

    return result;
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

void Graph::printMSTMatrix()
{
    for(int i = 0; i < adjListMST.size(); i++)
    {
        std::cout << i << ": ";

        for (int j = 0; j < adjListMST[i].size(); j++)
        {
            std::cout << adjListMST[i][j] << " ";
        }
        
        std::cout << std::endl;
    }
}

std::vector<int> Graph::findOddDegreeVertices()
{
    std::vector<int> odds;
    
    for(int i = 0; i < size; i++)
    {
        if (adjListMST[i].size() % 2 != 0)
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

    // Find nodes of MST with odd degrees
    std::vector<int> odds = findOddDegreeVertices();

    while (!odds.empty()) 
    {
        int temp;
        int first = odds[0];
        weight = MAX;

        for (int i = 1; i < odds.size(); i++) 
        {
            int neighbourIndex = odds[i];

            if (adjacencyMatrix[first][neighbourIndex] < weight) 
            {
                weight = adjacencyMatrix[first][neighbourIndex];
                nearest = neighbourIndex;
                temp = i;
            }
        } 
        
        if (nearest != INVALID_VALUE)
        {
            adjListMST[first].push_back(nearest);
            adjListMST[nearest].push_back(first);
            odds.erase(odds.begin() + temp);
            odds.erase(odds.begin());
        }
    }
}

std::vector<int> Graph::getEulerianPath()
{
    std::vector<int> path;
    std::stack<int> stack;
    
    // We take the first element as starting vertex
    int currVertex = 0;

    while (!adjListMST[currVertex].empty())
    {
        // Add vertex to stack
        stack.push(currVertex);

        // Take the last vertex from the adjacency list
        int lastNeighbour = adjListMST[currVertex].back();
        // Remove the vertex and its neighbour from each others lists
        adjListMST[currVertex].pop_back();
        for (int i = adjListMST[lastNeighbour].size() - 1; i >=0; i--)
        {
            if (adjListMST[lastNeighbour][i] == currVertex)
            {
                adjListMST[lastNeighbour].erase(adjListMST[lastNeighbour].begin() + i);
                break;
            }
        }

        currVertex = lastNeighbour;
    }

    // FIXME: Why use stack? Is smth else better?
    while (!stack.empty())
    {
        path.push_back(currVertex);
        currVertex = stack.top();
        stack.pop();
    }

    path.push_back(currVertex);


    return path;
}

void Graph::makeHamiltonian(std::vector<int> &path)
{
    int pathSize = path.size();
    std::vector<bool> isVisited(pathSize, false);

    std::vector<int>::iterator it = path.begin();
    while (it != path.end())
    {
        if (isVisited[*it])
        {
            it = path.erase(it);
            continue;
        }

        isVisited[*it] = true;
        it += 1;
    }
}

int Graph::getPathLength(std::vector<int> path)
{
    int length = 0;
    int startingVertex = path[0];
    int vertexA, vertexB;

    for(int i = 0; i < path.size() - 1; i++)
    {
        vertexA = path[i];
        vertexB = path[i+1];
        length += adjacencyMatrix[vertexA][vertexB];
    }

    length += adjacencyMatrix[vertexB][startingVertex];

    return length;
}

void Graph::christofidesAlgorithm()
{
    std::vector<bool> isVisited(size, false);
    list adjList = findMST(adjacencyMatrix, isVisited, 0);
    setAdjListMST(adjList);
    perfectMatching();
    std::vector<int> path = getEulerianPath();
    makeHamiltonian(path);
    int length = getPathLength(path);

    std::cout << "Shortest path is " << length << std::endl;
}

void Graph::printPath(std::vector<int> path)
{
    for(int i = 0; i < path.size(); i++)
    {
        std::cout << path[i] << " ";
    }
    std::cout << std::endl;
}

int Graph::getMSTLength(list MST)
{
    int result = 0;

    for (int i = 0; i < MST.size(); i++)
    {
        for (int j = 0; j < MST[i].size(); j++)
        {
            vertex A = i;
            vertex B = MST[i][j];

            result += adjacencyMatrix[A][B];
        }
    }

    // We take every edge twice so we divide the result by 2
    return result / 2;
}

void Graph::removeFromMatrix(matrix &adjMatrix, vertex index)
{
    adjMatrix.erase(adjMatrix.begin() + index);
    
    int size = adjMatrix.size();

    for (int i = 0; i < size; i++)
    {
        adjMatrix[i].erase(adjMatrix[i].begin() + index);
    }
}

void Graph::removeFromAdjList(list &adjList, vertex index)
{
    adjList[index].clear();

    for(int i = 0; i < adjList.size(); i++)
    {
        for(int j = 0; j < adjList[i].size(); j++)
        {
            if (adjList[i][j] == index)
            {
                adjList[i].erase(adjList[i].begin() + j);
                break;
            }
        }
    }
}

// Updating the queue by removing the vertex with given index and returning its cost
// We will later push the vertex with the updated cost in the queue
int getCost(std::priority_queue<evPair, std::vector<evPair>, std::greater<evPair> > &queue, vertex index)
{
    std::priority_queue<evPair, std::vector<evPair>, std::greater<evPair> > temp;
    int cost = 0;

    while (!queue.empty())
    {
        if(queue.top().second == index)
        {
            cost = queue.top().first;
            queue.pop();
            continue;
        }

        temp.push(queue.top());
        queue.pop();
    }

    if (!temp.empty())
    {
        queue = temp;
    }

    return cost;
}

void Graph::aStarAlgorithm()
{
    std::priority_queue< evPair, 
                         std::vector<evPair>, 
                         std::greater<evPair> > opened;
    std::vector<vertex> closed;
    matrix adjMatrix = adjacencyMatrix;
    std::vector<bool> isVisited(size, false);
    vertex start = 0;
    list adjList = findMST(adjacencyMatrix, isVisited, start);
    printMSTMatrix();

    opened.push({start, 0});

    while(!opened.empty())
    {
        evPair current = opened.top();
        closed.push_back(current.second);
        isVisited[current.second] = true;
        opened.pop();
        
        // removeFromMatrix(adjMatrix, current.second);

        if (closed.size() == size){
            break;
        }

        std::cout << "curr.sec: " << current.second << std::endl;
        int s = adjList[current.second].size();
        std::cout << "adjList size: " << s << std::endl;
        for (int i = 0; i < s; i++)
        {
            adjListMST = findMST(adjMatrix, isVisited, current.second);
            printMSTMatrix();

            list MST = findMST(adjMatrix, isVisited, current.second);
            int estimator = getMSTLength(MST);
            int weight = adjMatrix[current.second][i];
            int cost = weight + estimator + getCost(opened, current.second);

            // std::cout << "adjList[curr.sec][i]" << 
            evPair pair;
            pair.first = cost;
            pair.second = adjList[current.second][i];
            opened.push(pair);
        }

        removeFromAdjList(adjList, current.second);
    }

    std::cout << "\n\nShortest path is: " << getPathLength(closed) << std::endl;
}


Graph::Graph(int _size, matrix _adjacencyMatrix) : size{_size}, adjacencyMatrix{_adjacencyMatrix} {}
