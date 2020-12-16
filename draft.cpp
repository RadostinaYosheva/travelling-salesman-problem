#include <iostream>
#include <vector>
#include <limits>
#include <map>

const int cityCount = 4;
const int INVALID_VALUE = -1;
const int MAX = std::numeric_limits<int>::max();
std::map<std::pair<int, int>, int> edgeMap;

const int distance[cityCount][cityCount] = {{0, 12, 30, 6},
                                            {12, 0, 10, 7},
                                            {30, 10, 0, 5},
                                            {6, 7, 5, 0}};

// TODO: init from a file
void initEdgeMap(){

    for(int i = 0; i < cityCount; i++) 
    {
        for (int j = i; j < cityCount; j++)
        {
            if (distance[i][j] == 0)
            {
                continue;
            }

            edgeMap[std::pair<int, int>(i, j)] = distance[i][j];
            edgeMap[std::pair<int, int>(j, i)] = distance[i][j];

        }
    }
}

void printEdgeMap() {
        printf("Edge map contents:\n");
    // Loop over edges in edge_map
    for(
        std::map<std::pair<int, int>, int>::iterator it = edgeMap.begin();
        it != edgeMap.end();
        it++ )
    {
        printf("(%d,%d) %d\n",
        it->first.first,
        it->first.second,
        it->second);
    }
}



// int distance[cityCount][cityCount] = {{ 0, 5, 10, 15 },
//                                         { 5, 0, 20, 30 },
//                                         { 10, 20, 0, 35 },
//                                         { 15, 30, 35, 0 }};

                                        
// int distance[cityCount][cityCount] = {{ 0, 10, 15, 20 },
//                                       { 10, 0, 35, 25 }, 
//                                       { 15, 35, 0, 30 }, 
//                                       { 20, 25, 30, 0 }}; 



/* Find the smallest distance between a given city and his neighbours */
int findMinDegreeIndex(int cityIndex, std::vector<bool> isNotVisited) 
{
    int min = MAX;
    int minIndex = INVALID_VALUE;

    for (int i = 0; i < cityCount; i++)
    {
        if (!isNotVisited[i])
        {
            continue;
        }
        
        // it doesn't matter if @city is first or second because the matrix is symmetric
        if (min > distance[cityIndex][i] && distance[cityIndex][i] != 0)
        {
            min = distance[cityIndex][i];
            minIndex = i;
        }
    }
    return minIndex;
}


/* Finding the shortest path from a given city */
int findShortestPath(int startingPoint) {

    int currentPoint = startingPoint;
    int shortest = 0;
    std::vector<bool> isNotVisited(cityCount, true);
    isNotVisited[startingPoint] = false;
    

    for(int i = 0; i < cityCount; i++) 
    {
        int nearestNeighbour = findMinDegreeIndex(currentPoint, isNotVisited);

        if (nearestNeighbour == INVALID_VALUE) {
            shortest += distance[startingPoint][currentPoint];
            break;
        }

        shortest += distance[currentPoint][nearestNeighbour];
        currentPoint = nearestNeighbour;
        isNotVisited[nearestNeighbour] = false;
    }
    
    return shortest;
}



int main()
{
    // int shortestPath = MAX;

    // // we can use for loop because we have to visit all cities
    // for (int i = 0; i < cityCount; i++) 
    // {
    //     int startingPoint = i;
    //     int currentPath = findShortestPath(startingPoint);

    //     shortestPath = std::min(findShortestPath(startingPoint), shortestPath);
    // }

    // std::cout << "Shortest path is " << shortestPath << "km long" << std::endl;

    initEdgeMap();
    printEdgeMap();

    return 0;
}
