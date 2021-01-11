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



int Graph::christofidesAlgorithm()
{
    vertex start = 0;
    std::vector<bool> isVisited(size, false);
    isVisited[start] = true;
    // FIXME: magic numbers
    list adjList = findMST(adjacencyMatrix, isVisited, start, 1);
    setAdjListMST(adjList);
    perfectMatching();
    std::vector<vertex> path = getEulerianPath();
    makeHamiltonian(path);

    return getPathLength(path);
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

std::vector<vertex> Graph::findOddDegreeVertices()
{
    std::vector<vertex> odds;
    
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
    vertex nearest = INVALID_VALUE;
    edgeWeight weight;

    // Find nodes of MST with odd degrees
    std::vector<vertex> odds = findOddDegreeVertices();

    while (!odds.empty()) 
    {
        int temp;
        vertex first = odds[0];
        weight = MAX;

        for (int i = 1; i < odds.size(); i++) 
        {
            vertex neighbourIndex = odds[i];

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
std::vector<vertex> Graph::getEulerianPath()
{
    std::vector<vertex> path;
    std::stack<vertex> stack;
    
    // We take the first element as starting vertex
    vertex currVertex = 0;

    while (!adjListMST[currVertex].empty())
    {
        // Add vertex to stack
        stack.push(currVertex);

        // Take the last vertex from the adjacency list
        vertex lastNeighbour = adjListMST[currVertex].back();
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

    // Using stack because we look for a path from the back of the vectors
    // (and for some color)
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
void Graph::makeHamiltonian(std::vector<vertex> &path)
{
    int pathSize = path.size();
    std::vector<bool> isVisited(pathSize, false);

    std::vector<vertex>::iterator it = path.begin();
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


int Graph::getMSTLength(list MST)
{
    int result = 0;

    for (int i = 0; i < MST.size(); i++)
    {
        for (int j = 0; j < MST[i].size(); j++)
        {
            vertex first = i;
            vertex second = MST[i][j];

            result += adjacencyMatrix[first][second];
        }
    }

    // We take every edge twice 
    // so we divide the result by 2
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
int Graph::getCost(std::priority_queue<edgeVertexPair, std::vector<edgeVertexPair>, std::greater<edgeVertexPair> > &queue, vertex index)
{
    std::priority_queue<edgeVertexPair, std::vector<edgeVertexPair>, std::greater<edgeVertexPair> > temp;
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
    std::priority_queue< edgeVertexPair, 
                         std::vector<edgeVertexPair>, 
                         std::greater<edgeVertexPair> > opened;
    std::vector<vertex> closed;
    std::vector<bool> isVisited(size, false);
    vertex start = 0;
    int numVisited = 0;
    // FIXME: magic numbers
    list adjList = findMST(adjacencyMatrix, isVisited, start, 1);

    opened.push({0, start});

    while(!opened.empty())
    {
        edgeVertexPair current = opened.top();
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
