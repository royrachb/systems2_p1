/*
 * Test program for Exercise 1.
 * Author: Benjamin Saldman.
 *
 * Edit: Sapir Dahan
 * ID: 325732972
 * Mail: sapirdahan2003@gmail.com
 */

#include "doctest.h"
#include "Algorithms.hpp"
#include "Graph.hpp"


using namespace std;

TEST_CASE("Test isConnected") {
    ariel::Graph g;
    // Define a valid graph
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}
    };
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isConnected(g) == true);
}





TEST_CASE("Test isContainsCycle"){
    ariel::Graph g;

    // Not contained cycle
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");

    // Contained cycle
    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isContainsCycle(g) != "0");
}

TEST_CASE("Test isBipartite")
{
    ariel::Graph g;

    // Bipartite
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2}, B={1}.");

    // Not bipartite
    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is not bipartite");

    // Bipartite
    vector<vector<int>> graph3 = {
        {0, 1, 0, 0, 0},
        {1, 0, 3, 0, 0},
        {0, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2, 4}, B={1, 3}.");
}

TEST_CASE("Test negative cycles"){
    ariel::Graph g;

    // Negative cycle
    vector<vector<int>> graph = {
        {0, 1, 2},
        {1, 0, -5},
        {2, -5, 0}};

    g.loadGraph(graph);
    CHECK(ariel::Algorithms::negativeCycle(g) == "The graph contains negative cycle");

    // Negative cycle not in path
    vector<vector<int>> graph2 = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 0, 2},
        {0, 0, -5, 0, 0}};

    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::negativeCycle(g) == "The graph contains negative cycle");

    // Negative cycle not found at first search dua to the graph not being connected
    vector<vector<int>> graph3 = {
        {0, 1, 0, 0, 0},
        {1, 0, 3, 0, 0},
        {0, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, -10, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::negativeCycle(g) == "The graph contains negative cycle");

    // Not contained negative cycle
 	vector<vector<int>> graph4 = {
        {0, 1, 0, 0, 0},
        {1, 0, 3, 0, 0},
        {0, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, -3, 0}};
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::negativeCycle(g) == "The graph NOT contains negative cycle");

	// Cycle sum 0
	vector<vector<int>> graph5 = {
        {0, 1, 0},
        {0, 0, 2},
        {-3, 0, 0}};

    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::negativeCycle(g) == "The graph NOT contains negative cycle");

}

TEST_CASE("Test shortestPath") {
    ariel::Graph g;

    // Strongly connected
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}
    };
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");

    // No path exists
    vector<vector<int>> graph2 = {
        {0, 1, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 1},
        {0, 0, 0, 0}
    };
    g.loadGraph(graph2);
    CHECK_THROWS(ariel::Algorithms::shortestPath(g, 0, 3));


    // Normal graph without negative cycles
    vector<vector<int>> graph4 = {
        {0, 1, 0},
        {0, 0, 1},
        {0, 0, 0}
    };
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");
}

TEST_CASE("Test invalid graph") {
    ariel::Graph g;

    // Non-square matrix
    vector<vector<int>> graph5 = {
        {0, 1},
        {1, 0, 1}
    };
    CHECK_THROWS(g.loadGraph(graph5));

    // Empty graph
    vector<vector<int>> graph6 = {};
    CHECK_THROWS(g.loadGraph(graph6));

    // Non-square matrix with extra column
    vector<vector<int>> graph7 = {
        {0, 1, 0},
        {1, 0}
    };
    CHECK_THROWS(g.loadGraph(graph7));
}

TEST_CASE("Test print graph"){
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2, 0, 0},
        {1, 0, 3, 0, 0},
        {2, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph);

    CHECK(g.printGraph() == "Graph with 5 vertices and 10 edges.");

    vector<vector<int>> graph1 = {
        {0, 1},
        {0, 0}};
    g.loadGraph(graph1);

    CHECK(g.printGraph() == "Graph with 2 vertices and 1 edges.");

    vector<vector<int>> graph2 = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);

    CHECK(g.printGraph() == "Graph with 5 vertices and 0 edges.");
}

TEST_CASE("Test graph size"){
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2, 0, 0},
        {1, 0, 3, 0, 0},
        {2, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph);

    CHECK(g.size() == 5);
}
