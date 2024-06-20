#include "Graph.hpp"
#include "Algorithms.hpp"
#include <iostream>

int main() {
    ariel::Graph g;
    std::vector<std::vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}
    };

    g.loadGraph(graph);
    std::cout << g.printGraph() << std::endl;

    if (ariel::Algorithms::isConnected(g)) {
        std::cout << "The graph is connected." << std::endl;
    } else {
        std::cout << "The graph is not connected." << std::endl;
    }

    return 0;
}
