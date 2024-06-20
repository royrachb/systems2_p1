//mail:royrachbuch@gmail.com
#include "Algorithms.hpp"
#include <vector>
#include <queue>
#include <stack>
#include <sstream>
#include <limits>
#include <algorithm>
#include <functional>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <climits>

using namespace std;

namespace ariel {

bool Algorithms::isConnected(const Graph& g) {
    std::vector<std::vector<int>> adjacencyMatrix = g.getAdjacencyMatrix();
    std::size_t n = adjacencyMatrix.size();

    // Check if all vertices are reachable from vertex 0
    std::vector<bool> visited(n, false);
    dfs(0, adjacencyMatrix, visited);
    for (bool vertexVisited : visited) {
        if (!vertexVisited) {
            return false;
        }
    }

    // Create the transpose of the graph
    std::vector<std::vector<int>> transposedAdjacencyMatrix(n, std::vector<int>(n, 0));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            if (adjacencyMatrix[i][j] != 0) {
                transposedAdjacencyMatrix[j][i] = adjacencyMatrix[i][j];
            }
        }
    }

    // Check if all vertices are reachable from vertex 0 in the transposed graph
    std::fill(visited.begin(), visited.end(), false);
    dfs(0, transposedAdjacencyMatrix, visited);
    for (bool vertexVisited : visited) {
        if (!vertexVisited) {
            return false;
        }
    }

    return true;
}


    void Algorithms::dfs(std::size_t vertex, const std::vector<std::vector<int>>& adjacencyMatrix, std::vector<bool>& visited) {
        visited[vertex] = true;
        for (std::size_t neighbor = 0; neighbor < adjacencyMatrix[vertex].size(); ++neighbor) {
            if (adjacencyMatrix[vertex][neighbor] != 0 && !visited[neighbor]) {
                dfs(neighbor, adjacencyMatrix, visited);
            }
        }
    }

std::string Algorithms::shortestPath(const Graph& g, int start, int end) {
    std::vector<std::vector<int>> adjacencyMatrix = g.getAdjacencyMatrix();
    std::size_t n = adjacencyMatrix.size();
    std::vector<int> dist(n, std::numeric_limits<int>::max());
    std::vector<int> prev(n, -1);

    dist[static_cast<std::size_t>(start)] = 0;

    // Bellman-Ford algorithm to detect negative cycles
    for (std::size_t i = 0; i < n - 1; ++i) {
        for (std::size_t u = 0; u < n; ++u) {
            for (std::size_t v = 0; v < n; ++v) {
                if (adjacencyMatrix[u][v] != 0 && dist[u] != std::numeric_limits<int>::max() &&
                    dist[u] + adjacencyMatrix[u][v] < dist[v]) {
                    dist[v] = dist[u] + adjacencyMatrix[u][v];
                    prev[v] = static_cast<int>(u);
                }
            }
        }
    }

    // Check for negative cycles
    for (std::size_t u = 0; u < n; ++u) {
        for (std::size_t v = 0; v < n; ++v) {
            if (adjacencyMatrix[u][v] != 0 && dist[u] != std::numeric_limits<int>::max() &&
                dist[u] + adjacencyMatrix[u][v] < dist[v]) {
                // Negative cycle detected
                return "Negative cycle detected in the path.";
            }
        }
    }

    // If the end node is unreachable
    if (dist[static_cast<std::size_t>(end)] == std::numeric_limits<int>::max()) {
        throw std::runtime_error("No path exists between the nodes");
    }

    std::vector<int> path;
    for (int at = end; at != -1; at = prev[static_cast<std::size_t>(at)]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    std::ostringstream oss;
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (i != 0) {
            oss << "->";
        }
        oss << path[i];
    }

    return oss.str();
}






bool Algorithms::dfsCycles(unsigned int startNode, unsigned int parentNode, const Graph& graph, std::vector<bool>& visited, std::vector<unsigned int>& parent, std::string& cyclePath) {
    std::stack<std::pair<unsigned int, unsigned int>> stack;
    stack.push(std::make_pair(startNode, parentNode));
    parent[startNode] = parentNode;

    while (!stack.empty()) {
        std::pair<unsigned int, unsigned int> nodePair = stack.top();
        unsigned int node = nodePair.first;
        unsigned int parentNode = nodePair.second;
        stack.pop();

        if (!visited[node]) {
            visited[node] = true;
            parent[node] = parentNode;
        }

        for (const unsigned int nextNode : graph.getConnectedVertices(node)) {
            if (!visited[nextNode]) {
                stack.push(std::make_pair(nextNode, node));
            } else if (nextNode != parent[node]) {
                cyclePath = std::to_string(nextNode);
                unsigned int current = node;
                while (current != nextNode) {
                    cyclePath += "->" + std::to_string(current);
                    current = parent[current];
                }
                cyclePath += "->" + std::to_string(nextNode);
                return true;
            }
        }
    }

    return false;
}



   std::string Algorithms::isContainsCycle(const Graph& graph) {
    const unsigned int numVertices = graph.size();
    std::vector<bool> visited(numVertices, false);
    std::vector<unsigned int> parent(numVertices, INT_MAX);

    for (unsigned int i = 0; i < numVertices; ++i) {
        if (!visited[i]) {
            std::string cyclePath;
            if (dfsCycles(i, INT_MAX, graph, visited, parent, cyclePath)) {
                if (countOccurrences(cyclePath, '>') >= 2) {
                    return "The cycle is: " + cyclePath;
                } else {
                    std::fill(visited.begin(), visited.end(), false);
                }
            }
        }
    }

    return "0";
}


 unsigned int Algorithms::countOccurrences(const std::string& str, char target) {
    return std::count(str.begin(), str.end(), target);
}

    std::string Algorithms::isBipartite(const Graph& g) {
    std::vector<std::vector<int>> adjacencyMatrix = g.getAdjacencyMatrix();
    std::size_t n = adjacencyMatrix.size();
    std::vector<int> color(n, -1);

    std::function<bool(int)> bfsCheck = [&](int start) {
        std::queue<int> q;
        q.push(start);
        color[static_cast<std::size_t>(start)] = 0;

        while (!q.empty()) {
            int u = q.front();
            q.pop();

            for (std::size_t v = 0; v < n; ++v) {
                if (adjacencyMatrix[static_cast<std::size_t>(u)][v] != 0) {
                    if (color[v] == -1) {
                        color[v] = 1 - color[static_cast<std::size_t>(u)];
                        q.push(static_cast<int>(v));
                    } else if (color[v] == color[static_cast<std::size_t>(u)]) {
                        return false;
                    }
                }
            }
        }
        return true;
    };

    for (std::size_t i = 0; i < n; ++i) {
        if (color[i] == -1 && !bfsCheck(static_cast<int>(i))) {
            return "The graph is not bipartite";
        }
    }

    std::vector<int> setA, setB;
    for (std::size_t i = 0; i < n; ++i) {
        if (color[i] == 0) {
            setA.push_back(static_cast<int>(i));
        } else if (color[i] == 1) {
            setB.push_back(static_cast<int>(i));
        }
    }

    std::sort(setA.begin(), setA.end());
    std::sort(setB.begin(), setB.end());

    std::ostringstream oss;
    oss << "The graph is bipartite: A={";
    for (std::size_t i = 0; i < setA.size(); ++i) {
        if (i != 0) {
            oss << ", ";
        }
        oss << setA[i];
    }
    oss << "}, B={";
    for (std::size_t i = 0; i < setB.size(); ++i) {
        if (i != 0) {
            oss << ", ";
        }
        oss << setB[i];
    }
    oss << "}.";

    return oss.str();
}


    std::string Algorithms::negativeCycle(const Graph& g) {
        std::vector<std::vector<int>> adjacencyMatrix = g.getAdjacencyMatrix();
        std::size_t n = adjacencyMatrix.size();
        std::vector<int> dist(n, std::numeric_limits<int>::max());
        dist[0] = 0;  // Assume source vertex is 0

        // Relax edges |V|-1 times
        for (std::size_t i = 1; i < n; ++i) {
            for (std::size_t u = 0; u < n; ++u) {
                for (std::size_t v = 0; v < n; ++v) {
                    if (adjacencyMatrix[u][v] != 0 && dist[u] != std::numeric_limits<int>::max() &&
                        dist[u] + adjacencyMatrix[u][v] < dist[v]) {
                        dist[v] = dist[u] + adjacencyMatrix[u][v];
                    }
                }
            }
        }

        // Check for negative weight cycles
        for (std::size_t u = 0; u < n; ++u) {
            for (std::size_t v = 0; v < n; ++v) {
                if (adjacencyMatrix[u][v] != 0 && dist[u] != std::numeric_limits<int>::max() &&
                    dist[u] + adjacencyMatrix[u][v] < dist[v]) {
                    return "The graph contains negative cycle";
                }
            }
        }

        return "The graph NOT contains negative cycle";
    }

}
