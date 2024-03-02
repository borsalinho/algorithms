#include <gtest/gtest.h>

#include "../graph/s21_graph.h"

class TestGraph : public testing::Test {
 protected:
  Graph graph;
};

TEST_F(TestGraph, InvalidFileTest) {
  std::string invalidFileName = "tests/files/invalid_file.txt";
  EXPECT_ANY_THROW(graph.LoadGraphFromFile(invalidFileName));
}

TEST_F(TestGraph, ParsingErrorTest) {
  EXPECT_ANY_THROW(
      graph.LoadGraphFromFile("tests/files/parsing_error_file.txt"));
}

TEST_F(TestGraph, EmptyFileTest) {
  EXPECT_ANY_THROW(graph.LoadGraphFromFile("tests/files/empty_file.txt"));
}

TEST_F(TestGraph, AlmostEmptyFileTest) {
  EXPECT_ANY_THROW(
      graph.LoadGraphFromFile("tests/files/almost_empty_file.txt"));
}

TEST_F(TestGraph, MatrixSizeMismatchTest1) {
  EXPECT_ANY_THROW(graph.LoadGraphFromFile("tests/files/wrong_matrix.txt"));
}

TEST_F(TestGraph, MatrixSizeMismatchTest2) {
  EXPECT_ANY_THROW(graph.LoadGraphFromFile("tests/files/mismatch_matrix.txt"));
}

TEST_F(TestGraph, NegativeNumbersTest) {
  EXPECT_ANY_THROW(graph.LoadGraphFromFile("tests/files/negative_numbers.txt"));
}

TEST_F(TestGraph, ExtraLineAfterMatrixTest) {
  std::string filename = "tests/files/extra_line_after_matrix.txt";
  EXPECT_ANY_THROW(graph.LoadGraphFromFile(filename));
}

TEST_F(TestGraph, EmptyFileTest2) { EXPECT_ANY_THROW(graph.ToString()); }