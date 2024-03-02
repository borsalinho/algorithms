#include <gtest/gtest.h>

#include "../graphAlgorithms/s21_graphAlgorithms.h"

using Matrix = std::vector<std::vector<int>>;

const int infinity = -1;

TEST(ShortPathsAllVertices, Test2x2) {
  Graph graph;

  graph.LoadGraphFromFile("tests/files/graph_2x2.txt");

  GraphAlgorithms algorithm;
  Matrix distanceMatrix = algorithm.GetShortestPathsBetweenAllVertices(graph);

  Matrix expected = {{0, 1}, {1, 0}};

  ASSERT_EQ(distanceMatrix, expected);
}

TEST(ShortPathsAllVertices, Test3x3) {
  Graph graph;
  graph.LoadGraphFromFile("tests/files/digraph_3x3.txt");

  GraphAlgorithms algorithm;
  Matrix distanceMatrix = algorithm.GetShortestPathsBetweenAllVertices(graph);

  Matrix expected = {
      {1, 2, infinity}, {infinity, 1, infinity}, {infinity, infinity, 1}};

  ASSERT_EQ(distanceMatrix, expected);
}
