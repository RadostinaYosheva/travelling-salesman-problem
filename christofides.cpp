#include "graph.h"
#include <limits>
#include <stack>

const int INVALID_VALUE = -1;
const int MAX = std::numeric_limits<int>::max();



int Graph::christofidesAlgorithm()
{
    vertex start = 0;
    std::vector<bool> isVisited(size, false);
    isVisited[start] = true;
    list adjList = findMST(adjacencyMatrix, isVisited, start, 1);
    setAdjListMST(adjList);
    perfectMatching();
    std::vector<vertex> path = getEulerianPath();
    makeHamiltonian(path);

    return getPathLength(path);
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
