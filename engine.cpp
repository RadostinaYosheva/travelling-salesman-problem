#include "engine.h"
#include "timer.h"
#include <fstream>
#include <sys/time.h>
#include <sys/resource.h>

void Engine::run()
{
    int algorithm = chooseAlgorithms();

    while (algorithm > 0 && algorithm < 5)
    {
        int test = chooseTestCases();

        while (test > 0 && test < 6)
        {
            std::string fileName = generateTestCase(test);
            Graph g = init(fileName);
            
            timerTest(g, algorithm);
            
            test = chooseTestCases();
        }

        algorithm = chooseAlgorithms();
    }
}

int Engine::chooseTestCases()
{
    int choice;

    for (int i = 1; i <= 5; i++)
    {
        printf("\n%d. Test %d", i, i);
    }

    std::cout << "\n6. Change algorithm" << std::endl;
    std::cout << "\nPress any key to exit" << std::endl;

    std::cout << "\nChoice: ";
    std::cin >> choice;

    return choice;
}

int Engine::chooseAlgorithms()
{
    int choice;

    std::cout << "\n1. Nearest Neighbour Algorithm" << std::endl;
    std::cout << "2. Chrostofides' Algorithm" << std::endl;
    std::cout << "3. A* (A Star) Algorithm" << std::endl;
    std::cout << "Press any key to exit" << std::endl;

    std::cout << "\nChoice: ";
    std::cin >> choice;

    return choice;
}

std::string Engine::generateTestCase(int choice)
{
    return "tests/test" + std::to_string(choice) + ".txt";
}

Graph Engine::init(std::string fileName)
{
    int size;
    std::vector<std::vector<int>> matrix;
    std::ifstream file(fileName);

    if (file.is_open())
    {
        file >> size;

        matrix.resize(size, std::vector<int>(size));

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                file >> matrix[i][j];
            }
        }
    }

    Graph graph(size, matrix);

    return graph;
}

long Engine::getMemoryUsage()
{
    struct rusage myUsage;

    getrusage(RUSAGE_SELF, &myUsage);

    return myUsage.ru_maxrss;
}

void Engine::timerTest(Graph g, int algorithm) {
    const int testRepeat = 1 << 10;

	uint64_t t0;
	uint64_t t1;

    int shortestPath;
    long memory;

	{
		t0 = timer_nsec();
		for (int test = 0; test < testRepeat; ++test) {
            switch (algorithm)
            {
            case 1:
                shortestPath = g.nearestNeighbourAlgorithm();
                memory = getMemoryUsage();
                break;
            case 2:
                shortestPath = g.christofidesAlgorithm();
                memory = getMemoryUsage();
                break;

            case 3:
                shortestPath = g.aStarAlgorithm();
                memory = getMemoryUsage();
                break;

            default:
                return;
            }
		}
		t1 = timer_nsec();
	}

    const double averageTime = (double(t1 - t0) * 1e-9) / testRepeat;
    printf("\nShortest path: %d\n", shortestPath);
	printf("Time: %f\n", averageTime);
    printf("Memory: %ld\n", memory);
}

Engine::Engine() {}