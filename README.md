# Travelling Salesman Problem

---

A university assignment to implement at least two heuristic algoritms to solve the TSP.
By Radostina Yosheva, Computer Science strudent at FMI, Sofia Univercity "St. Kliment Ohridski".
This project has a short comparitive analysis in bulgarian.

## The problem
Given a collection of vertices connected by edges and the weight of every edge, what is the shortest route that visits every vertex and returns to the starting point?<br/>

## Used Algorithms
1. __Nearest Neighbour__
The salesman starts at a random city and repeatedly visits the nearest city until all have been visited. The algorithm quickly yields a short tour, but usually not the optimal one.
Worst case performance: $$Θ(n^2)$$

2. __Chrisofides' Algorithm__
This is an algorithm for finding an approximate solution to the problem using Minimum Spanning Tree, using the Prim's algorithm, finding the perfect matching of the tree and getting the Euler's path for shortcuts.

3. __A* Algorithm__
A* can be seen as an extension of Dijkstra's algorithm but it achieves better performance by using heuristics to guide its search. The heuristic used for the problem is the length of the MST of the unvisited vertices.
Worst case performance: $$O(|E|)$$

## Future development
Since the assignement was for Data Structures and Programming course, the OOP structure was not regarded. The architecture should be fixed and tests and documentation should be added.
There won't be any further realizations of the project because it is only implementations of already existing algorithms.

## To run the project:
- Write the command ```make``` and then ```./output```.
- From the menu chose an algorithm and enter its number.
- Chose a test and enter its number of enter 6 to change the angorithm.
- To exit press any key.

### Technology Stack
The project is written in C++ with Visual Studio Code as IDE.
