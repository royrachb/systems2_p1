//mail:royrachbuch@gmail.com
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <string>

namespace ariel {

    class Graph {
    private:
        std::size_t vertices;
        std::size_t edges;
        std::vector<std::vector<int>> adjacencyMatrix;

    public:
        Graph();
        void loadGraph(const std::vector<std::vector<int>>& matrix);
        std::vector<std::vector<int>> getAdjacencyMatrix() const;
        std::size_t size() const;
        std::vector<unsigned int> getConnectedVertices(unsigned int node) const;
        std::string printGraph() const; // Declaration
    };

} // namespace ariel

#endif // GRAPH_HPP
