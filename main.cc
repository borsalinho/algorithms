#include <iostream>

#include "graph/s21_graph.h"
#include "graphAlgorithms/s21_graphAlgorithms.h"
#include "menu/menu.h"

int main() {
  Graph obj;

  Menu menu;

  menu.ShowMenu();

  // try{
  //     obj.LoadGraphFromFile("./digraph_6x6_AntAlg.txt");
  //     // obj.ExportGraphToDot("./digraph_5x5");

  //     GraphAlgorithms alg;
  //     // alg.DepthFirstSearch(obj,1);

  //     for(auto &el :  alg.DepthFirstSearch(obj,1)){
  //         // std::cout << el << ' ';
  //     }

  //     // std::cout << '\n';
  //     for(auto &el :  alg.BreadthFirstSearch(obj,1)){
  //         // std::cout << el << ' ';
  //     }

  //     // std::cout << '\n';
  //     std::cout << "я GetShortestPathBetweenVertices = " <<
  //     alg.GetShortestPathBetweenVertices(obj, 1,5) << std::endl; std::cout <<
  //     '\n';

  //     // std::cout << "я GetShortestPathsBetweenAllVertices = " << std::endl;
  //     for(auto &el :  alg.GetShortestPathsBetweenAllVertices(obj)){
  //         for(auto &e :  el){
  //             // std::cout << e << ' ';
  //         }
  //         // std::cout << '\n';
  //     }

  //     // std::cout << "я GetLeastSpanningTree = " << std::endl;
  //     for(auto &el :  alg.GetLeastSpanningTree(obj)){
  //         for(auto &e :  el){
  //             // std::cout << e << ' ';
  //         }
  //         // std::cout << '\n';
  //     }

  //     // std::cout << "я SolveTravelingSalesmanProblem = " << std::endl;
  //     auto ress = alg.SolveTravelingSalesmanProblem(obj);
  //     for(auto &el : ress.vertices){
  //         std::cout << el << " ";
  //     }
  //     std::cout << '\n';
  //     std::cout << "distance = "<<
  //     alg.SolveTravelingSalesmanProblem(obj).distance <<'\n';

  // } catch(char const* q){
  //     std::cout << q << std::endl;
  // }

  return 0;
}