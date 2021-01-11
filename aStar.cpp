#include "graph.h"


int Graph::aStarAlgorithm()
{
    std::priority_queue< edgeVertexPair, 
                         std::vector<edgeVertexPair>, 
                         std::greater<edgeVertexPair> > opened;
    std::vector<vertex> closed;
    std::vector<bool> isVisited(size, false);
    vertex start = 0;
    int numVisited = 0;
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
