#include "graph.h"
#include <fstream>

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

int main()
{
    Graph g;
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
            g = init("tests/test1.txt");
            break;

        case 2:
            g = init("tests/test2.txt");
            break;

        case 3:
            g = init("tests/test3.txt");
            break;
        
        default:
            return 0;
        }

        g.nearestNeighbour();
    }

    return 0;
}