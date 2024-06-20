//mail:royrachbuch@gmail.com
#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP

#include "Graph.hpp"
#include <string>
#include <vector>

namespace ariel {

    class Algorithms {
    public:
        static bool isConnected(const Graph& g);
        static std::string shortestPath(const Graph& g, int start, int end);
        static std::string isContainsCycle(const Graph& graph);
        static std::string isBipartite(const Graph& g);
        static std::string negativeCycle(const Graph& g);

        static unsigned int countOccurrences(const std::string& str, char target);
        static bool dfsCycles(unsigned int node, unsigned int parentNode, const Graph& graph, std::vector<bool>& visited, std::vector<unsigned int>& parent, std::string& cyclePath);

    private:
        static void dfs(std::size_t vertex, const std::vector<std::vector<int>>& adjacencyMatrix, std::vector<bool>& visited);
    };

} // namespace ariel

#endif // ALGORITHMS_HPP
