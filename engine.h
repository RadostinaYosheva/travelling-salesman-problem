#ifndef _ENGINE_H
#define _ENGINE_H

#include "graph.h"
#include <functional>

class Engine
{
    public:
    void run();
    Engine();

    private:
    int chooseAlgorithms();
    int chooseTestCases();
    std::string generateTestCase(int choice);
    Graph init(std::string fileName);
    // TODO: try with passing a function (std::function)
    void timerTest(Graph g, int algorithm);
    long getMemoryUsage();
};

#endif
