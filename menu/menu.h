#ifndef _MENU_MENU_H_
#define _MENU_MENU_H_

#include <iostream>
#include <vector>

#include "../graph/s21_graph.h"
#include "../graphAlgorithms/s21_graphAlgorithms.h"

class Menu {
 public:
  Menu() : graph_(), alg_(), choose_(0), exit_(7){};

  void ShowMenu() {
    LoadFile();
    while (choose_ != exit_) {
      ShowOption();
      ChooseOption();
      DoOptions();
    }
  }

 private:
  Graph graph_;
  GraphAlgorithms alg_;
  int choose_;
  int exit_;

  void DoOptions() {
    switch (choose_) {
      case 1:
        DoBreadthFirstSearch();
        break;
      case 2:
        DoDepthFirstSearch();
        break;
      case 3:
        DoShortestPathBetweenVertices();
        break;
      case 4:
        DoShortestPathsBetweenAllVertices();
        break;
      case 5:
        DoLeastSpanningTree();
        break;
      case 6:
        DoSolveTravelingSalesmanProblem();
        break;
      case 7:
        // exit
        break;
    }
  }

  void DoBreadthFirstSearch() {
    for (auto &el : alg_.BreadthFirstSearch(graph_, 1)) {
      std::cout << el << ' ';
    }
    std::cout << '\n';
  }

  void DoDepthFirstSearch() {
    for (auto &el : alg_.DepthFirstSearch(graph_, 1)) {
      std::cout << el << ' ';
    }
    std::cout << '\n';
  }

  void DoShortestPathBetweenVertices() {
    std::cout << "choose two points and set\n";
    auto points = alg_.DepthFirstSearch(graph_, 1);

    for (auto &el : points) {
      std::cout << el << ' ';
    }
    std::cout << '\n';

    std::cout << "enter the first point\n";
    int x = GetOptions(points);

    std::cout << "enter the second point\n";
    int y = GetOptions(points);

    std::cout << alg_.GetShortestPathBetweenVertices(graph_, x, y) << std::endl;
    std::cout << '\n';
  }

  void DoShortestPathsBetweenAllVertices() {
    for (auto &el : alg_.GetShortestPathsBetweenAllVertices(graph_)) {
      for (auto &e : el) {
        std::cout << e << ' ';
      }
      std::cout << '\n';
    }
  }

  void DoLeastSpanningTree() {
    for (auto &el : alg_.GetLeastSpanningTree(graph_)) {
      for (auto &e : el) {
        std::cout << e << ' ';
      }
      std::cout << '\n';
    }
  }

  void DoSolveTravelingSalesmanProblem() {
    auto ress = alg_.SolveTravelingSalesmanProblem(graph_);

    if (ress.vertices.empty()) {
      std::cout << "It is impossible to solve the problem with this graph\n";
      return;
    }

    for (auto &el : ress.vertices) {
      std::cout << el << " ";
    }

    std::cout << '\n';
    std::cout << "distance = " << ress.distance << '\n';
  }

  void ChooseOption() {
    std::string enter;
    while (1) {
      std::cin >> enter;

      bool all_digits = true;

      for (char c : enter) {
        if (!std::isdigit(c)) {
          all_digits = false;
          break;
        }
      }

      if (!all_digits || std::stoi(enter) < 1 || std::stoi(enter) > 7) {
        std::cout << "Shao Khan requires you to go from 1 to 7(c)"
                  << "\n";
        continue;
      } else {
        choose_ = std::stoi(enter);
        std::cout << "u choosed " << choose_ << '\n';
        break;
      }
    }
    std::cout << '\n';
    std::cout << "result\n";
  }

  void ShowOption() {
    std::cout << "Choose ur destiny(c)"
              << "\n";
    std::cout << "\n";
    std::cout << "1. graph traversal in breadth with output of the result to "
                 "the console"
              << "\n";
    std::cout << "2. graph traversal in depth with output of the result to the "
                 "console"
              << "\n";
    std::cout << "3. searching for the shortest path between any two vertices "
                 "and displaying the result to the console"
              << "\n";
    std::cout
        << "4. searching for the shortest paths between all pairs of vertices "
           "in the graph with the output of the resulting matrix to the console"
        << "\n";
    std::cout << "5. searching for the minimal spanning tree in the graph with "
                 "the output of the resulting adjacency matrix to the console"
              << "\n";
    std::cout << "6. solving the salesman problem with the output of the "
                 "resulting route and its length to the console"
              << "\n";
    std::cout << "7. exit"
              << "\n";
  }

  void LoadFile() {
    std::string enter;

    std::cout << "loading the original graph from a file\n";

    while (1) {
      std::cin >> enter;

      try {
        graph_.LoadGraphFromFile(enter);
        std::cout << "file fainded\n";
        break;
      } catch (const char *err) {
        std::cout << err << "\n";
        std::cout << "try again\n";
      }
    }
  }

  int GetOptions(std::vector<int> &points) {
    std::string res;
    int num;
    while (1) {
      std::cin >> res;
      num = std::stoi(res);
      for (auto &el : points) {
        if (el == num) return num;
        ;
      }
      std::cout << "need to select from the list\n";
    }

    return num;
  }
};

#endif  // _MENU_MENU_H_
