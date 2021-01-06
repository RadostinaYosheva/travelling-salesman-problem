#include "graph.h"
#include <fstream>

//  TODO: Find a place for this function (not in main)
Graph init(std::string fileName)
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

// TODO: Make an engine class and in main have one line "engine.run();"
int main()
{
    int choice = 0;

    while (choice != 4) {
        std::cout << std::endl;
        std::cout << "1. Test1" << std::endl;
        std::cout << "2. Test2" << std::endl;
        std::cout << "3. Test3" << std::endl;
        std::cout << "4. Exit" << std::endl;

        std::cin >> choice;

        switch (choice)
        {
            case 1:
            {
                Graph g = init("tests/test1.txt");
                g.nearestNeighbourAlgorithm();
                break;
            };

            case 2:
            {
                Graph g = init("tests/test2.txt");
                g.nearestNeighbourAlgorithm();
                break;
            };

            case 3:
            {
                Graph g = init("tests/test3.txt");
                g.nearestNeighbourAlgorithm();
                break;
            };
        
            default:
                return 0;
        }

        
    }

    return 0;
}