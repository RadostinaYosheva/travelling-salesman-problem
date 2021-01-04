#include <iostream>
#include <vector>
#include <limits>
#include <map>

// Timer
static uint64_t timer_nsec() {

#if defined(CLOCK_MONOTONIC_RAW)
	const clockid_t clockid = CLOCK_MONOTONIC_RAW;

#else
	const clockid_t clockid = CLOCK_MONOTONIC;

#endif

	timespec t;
	clock_gettime(clockid, &t);

	return t.tv_sec * 1000000000UL + t.tv_nsec;
}


// Constants used in the program
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
// FIXME: change key name to city or smth
int findMinDegreeIndex(int key, std::vector<bool> notVisited) 
{
    // TODO: Can we jump directly to key and then start iterate?
    
    int min = MAX;
    int minIndex = INVALID_VALUE;

    for(auto it : edgeMap) 
    {
        int currentCity = it.first.first;

        if (currentCity != key)
        {
            continue;
        }

        int neighbour = it.first.second;
        int distance = it.second;

        if (distance < min && notVisited[neighbour])
        {
            min = distance;
            minIndex = neighbour;
        }
    }

    return minIndex;
}


/* Finding the shortest path from a given city */
int findShortestPath(int startingPoint) {

    int currentPoint = startingPoint;
    int shortestPath = 0;
    std::vector<bool> isNotVisited(cityCount, true);
    isNotVisited[startingPoint] = false;
    

    for(int i = 0; i < cityCount; i++) 
    {
        int nearestNeighbour = findMinDegreeIndex(currentPoint, isNotVisited);

        if (nearestNeighbour == INVALID_VALUE) {
            shortestPath += edgeMap[{currentPoint, startingPoint}];    
            break;
        }

        shortestPath += edgeMap[{currentPoint, nearestNeighbour}];
        currentPoint = nearestNeighbour;
        isNotVisited[nearestNeighbour] = false;
    }

    return shortestPath;
}


// Gets the shortest path by changing the starting point
void getShortestPath() {
    int shortestPath = MAX;

    // we can use for loop because we have to visit all cities
    for (int i = 0; i < cityCount; i++) 
    {
        int startingPoint = i;
        int currentPath = findShortestPath(startingPoint);

        shortestPath = std::min(findShortestPath(startingPoint), shortestPath);
    }

    // std::cout << "Shortest path is " << shortestPath << "km long" << std::endl;
}


// Timer is only for Linux
void timerTest() {
    const int testRepeat = 1 << 10;

	uint64_t t0;
	uint64_t t1;

	{
		t0 = timer_nsec();
		for (int test = 0; test < testRepeat; ++test) {
            getShortestPath();
		}
		t1 = timer_nsec();
	}

    const double averageTime = (double(t1 - t0) * 1e-9) / testRepeat;
	printf("time %f\n", averageTime);
}



int main()
{
    initEdgeMap();
    printEdgeMap();

    return 0;
}
