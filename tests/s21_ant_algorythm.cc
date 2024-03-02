#include <gtest/gtest.h>

#include "../graphAlgorithms/s21_graphAlgorithms.h"

TEST(AntAlgorythm, test1) {
  Graph graph;
  GraphAlgorithms test;
  graph.LoadGraphFromFile("tests/files/digraph_6x6_AntAlg.txt");
  TsmResult result = test.SolveTravelingSalesmanProblem(graph);
  std::vector<size_t> way = {1, 2, 6, 3, 4, 5};
  double length = 700;
  // ASSERT_EQ(result.vertices, way);
  ASSERT_EQ(result.distance, length);
}

TEST(AntAlgorythm, test2) {
  Graph graph;
  GraphAlgorithms test;
  graph.LoadGraphFromFile("tests/files/digraph_12x12_AntAlg.txt");
  TsmResult result = test.SolveTravelingSalesmanProblem(graph);
  std::vector<int> way = {1, 7, 8, 9, 10, 2, 6, 3, 11, 12, 4, 5, 1};
  double length = 933;

  ASSERT_EQ(result.distance, length);
  ASSERT_EQ(result.vertices, way);
}