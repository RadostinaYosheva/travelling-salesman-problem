#include <limits>
#include <algorithm>
#include <stack>
#include "graph.h"

const int INVALID_VALUE = -1;
const int MAX = std::numeric_limits<int>::max();



int Graph::nearestNeighbourAlgorithm() 
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

    return shortestPath;
}


/* Finding the shortest path from a given city */
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


/* Find the index of the nearest neighbour*/
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



int Graph::christofidesAlgorithm()
{
    std::vector<bool> isVisited(size, false);
    // FIXME: magic numbers
    list adjList = findMST(adjacencyMatrix, isVisited, 0, 1);
    setAdjListMST(adjList);
    perfectMatching();
    std::vector<int> path = getEulerianPath();
    makeHamiltonian(path);

    return getPathLength(path);
}


/* Removes all edges where second vertex is the last visited vertex */
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

/* Adds all unvisited edges of given vertex to priority queue */
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

/* Function that returns a MinimumSpanningTree of a graph */
list Graph::findMST(matrix adjMatrix, std::vector<bool> isVisited, vertex start, int numVisited)
{
    int matrixSize = adjMatrix.size();
    list result(matrixSize);
    priorityQ queue;
    std::vector<int> visitedIndices;

    isVisited[start] = true;
    visitedIndices.push_back(start);


    // Why i < this ??
    for (int i = 0; i < matrixSize-numVisited; i++) 
    {
        for(int v : visitedIndices)
        {
            // Adding v's edges to priority queue
            addToPQ(queue, v, isVisited);
        }

        int keyVertex = queue.top().second.first;
        int nearest = queue.top().second.second;
        int weight = queue.top().first;

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

/* Matches all the vertices with odd degree */
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


/* Finds the path where every edge of the MST is visited */
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

    // FIXME: Why use stack? Is smth else better? -> vector because the order of the cities in the path does not matter
    while (!stack.empty())
    {
        path.push_back(currVertex);
        currVertex = stack.top();
        stack.pop();
    }

    path.push_back(currVertex);


    return path;
}


/* Takes shortcuts in path (Removes repeating vertices) */
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

/* Removes vertex from adjacency list */
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
int Graph::getCost(std::priority_queue<evPair, std::vector<evPair>, std::greater<evPair> > &queue, vertex index)
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

int Graph::aStarAlgorithm()
{
    std::priority_queue< evPair, 
                         std::vector<evPair>, 
                         std::greater<evPair> > opened;
    std::vector<vertex> closed;
    std::vector<bool> isVisited(size, false);
    vertex start = 0;
    int numVisited = 0;
    // FIXME: magic numbers
    list adjList = findMST(adjacencyMatrix, isVisited, start, 1);

    opened.push({0, start});

    while(!opened.empty())
    {
        evPair current = opened.top();
        closed.push_back(current.second);
        isVisited[current.second] = true;
        numVisited++;
        opened.pop();
        
        if (closed.size() == size){
            break;
        }

        for (int i = 0; i < adjList[current.second].size(); i++)
        {
            list MST = findMST(adjacencyMatrix, isVisited, current.second, numVisited);
            int estimator = getMSTLength(MST);
            int weight = adjacencyMatrix[current.second][i];
            int cost = weight + estimator + getCost(opened, current.second);

            opened.push( {cost, adjList[current.second][i]} );
        }

        removeFromAdjList(adjList, current.second);
    }

    return getPathLength(closed);
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
