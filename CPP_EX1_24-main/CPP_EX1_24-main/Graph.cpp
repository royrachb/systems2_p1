//mail:royrachbuch@gmail.com
#include "Graph.hpp"
#include <sstream>
#include <stdexcept>

namespace ariel {

    Graph::Graph() : vertices(0), edges(0) {}

   void Graph::loadGraph(const std::vector<std::vector<int>>& matrix) {
    std::size_t rows = matrix.size();
    if (rows == 0) {
        throw std::invalid_argument("Graph must have at least one vertex");
    }

    for (const auto& row : matrix) {
        if (row.size() != rows) {
            throw std::invalid_argument("Graph must be a square matrix");
        }
    }

    adjacencyMatrix = matrix;
    vertices = rows;
    edges = 0;
    for (const auto& row : matrix) {
        for (int val : row) {
            if (val != 0) {
                ++edges;
            }
        }
    }
}


    std::vector<std::vector<int>> Graph::getAdjacencyMatrix() const {
        return adjacencyMatrix;
    }

    std::size_t Graph::size() const {
        return vertices;
    }

    std::vector<unsigned int> Graph::getConnectedVertices(unsigned int node) const {
        std::vector<unsigned int> connectedVertices;
        for (unsigned int i = 0; i < adjacencyMatrix[node].size(); ++i) {
            if (adjacencyMatrix[node][i] != 0) {
                connectedVertices.push_back(i);
            }
        }
        return connectedVertices;
    }

    std::string Graph::printGraph() const {
        std::ostringstream oss;
        oss << "Graph with " << vertices << " vertices and " << edges << " edges.";
        return oss.str();
    }

} // namespace ariel
