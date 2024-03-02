#ifndef _GRAPH_ALGORITHMS_H
#define _GRAPH_ALGORITHMS_H

#include <climits>
#include <random>
#include <set>
#include <vector>

#include "../containers/queue/s21_queue.h"
#include "../containers/stack/s21_stack.h"
#include "../graph/s21_graph.h"

struct Edge {
  int from;
  int to;
  int weight;
};

struct TsmResult {
  std::vector<int>
      vertices;  // массив с искомым маршрутом (с порядком обхода вершин).
                 // Вместо int* можно использовать std::vector<int>
  double distance;  // длина этого маршрута
};

class GraphAlgorithms {
 public:
  GraphAlgorithms(){};
  ~GraphAlgorithms(){};

  std::vector<int> DepthFirstSearch(Graph &graph, int start_vertex) {
    std::vector<int> res;
    S21::stack<int> stack;

    stack.push(start_vertex);

    while (!stack.empty()) {
      int current = stack.top();
      stack.pop();

      if (!HasElement(res, current)) {
        res.push_back(current);

        for (int i = graph.GetSize(); i > 0; --i) {
          bool has_path = graph.GetMatrix()[current - 1][i - 1] != 0;

          if (has_path && !HasElement(res, i)) {
            stack.push(i);
          }
        }
      }
    }

    return res;
  }

  std::vector<int> BreadthFirstSearch(Graph &graph, int start_vertex) {
    std::vector<int> way;
    std::vector<int> res;
    S21::queue<int> queue;

    res.push_back(start_vertex);
    queue.push(start_vertex);

    while (!queue.empty()) {
      for (int i = 1; i <= graph.GetSize(); i++) {
        int edge = graph.GetMatrix()[queue.front() - 1][i - 1];
        if ((edge) && (!HasElement(res, i))) {
          queue.push(i);
          res.push_back(i);
        }
      }
      way.push_back(queue.front());
      queue.pop();
    }

    return res;
  }

  // 2
  int GetShortestPathBetweenVertices(Graph &graph, int vertex1, int vertex2) {
    int size = graph.GetSize();

    if (vertex1 > size || vertex2 > size || vertex1 < 1 || vertex2 < 1)
      throw "Invalid number entered\n";

    std::vector<int> distance(graph.GetSize(), INT_MAX);
    std::set<std::pair<int, int>> save;
    int start = vertex1 - 1;
    int finish = vertex2 - 1;
    distance[start] = 0;

    save.insert({distance[start], start});

    while (!save.empty()) {
      int start = save.begin()->second;
      save.erase(save.begin());

      for (int i = 0; i < size; i++) {
        int edge = graph.GetMatrix()[start][i];

        if (edge) {
          int new_lenght = distance[start] + edge;

          if (new_lenght < distance[i]) {
            save.erase({distance[i], i});
            distance[i] = new_lenght;
            save.insert({distance[i], i});
          }
        }
      }
    }
    return distance[finish];
  }

  std::vector<std::vector<int>> GetShortestPathsBetweenAllVertices(
      Graph &graph) {
    int inf = -1;
    std::vector<std::vector<int>> res = graph.GetMatrix();

    for (int row = 0; row < graph.GetSize(); ++row) {
      for (int col = 0; col < graph.GetSize(); ++col) {
        if (row != col && res[row][col] == 0) {
          res[row][col] = inf;
        }
      }
    }

    for (int i = 0; i < graph.GetSize(); ++i) {
      for (int j = 0; j < graph.GetSize(); ++j) {
        for (int k = 0; k < graph.GetSize(); ++k) {
          int src_dest = res[j][k];
          int src_middle = res[j][i];
          int middle_dest = res[i][k];
          int weight = src_middle + middle_dest;

          if (src_middle != inf && middle_dest != inf && weight < src_dest) {
            res[j][k] = weight;
          }
        }
      }
    }

    return res;
  }

  // 3
  std::vector<std::vector<int>> GetLeastSpanningTree(Graph &graph) {
    Edge edge{-1, -1, std::numeric_limits<int>::max()};

    std::vector<std::vector<int>> res(graph.GetSize(),
                                      std::vector<int>(graph.GetSize(), 0));
    std::vector<std::vector<int>> origin_matrix = graph.GetMatrix();
    std::vector<int> cheapest_cost(graph.GetSize(),
                                   std::numeric_limits<int>::max());
    std::vector<Edge> cheapest_edge(graph.GetSize(), edge);
    std::vector<Edge> spanning_tree(graph.GetSize() - 1, edge);
    std::vector<bool> in_tree(graph.GetSize(), false);

    int startVertex = 0;
    cheapest_cost[startVertex] = 0;

    for (int i = 0; i < graph.GetSize(); ++i) {
      int current_vertex = graph.GetSize();

      for (int v = 0; v < graph.GetSize(); ++v) {
        if (!in_tree[v] && (current_vertex == graph.GetSize() ||
                            cheapest_cost[v] < cheapest_cost[current_vertex])) {
          current_vertex = v;
        }
      }

      in_tree[current_vertex] = true;

      for (int neighbor = 0; neighbor < graph.GetSize(); ++neighbor) {
        if (origin_matrix[current_vertex][neighbor] > 0 && !in_tree[neighbor] &&
            origin_matrix[current_vertex][neighbor] < cheapest_cost[neighbor]) {
          cheapest_cost[neighbor] = origin_matrix[current_vertex][neighbor];
          Edge el{current_vertex, neighbor,
                  origin_matrix[current_vertex][neighbor]};
          cheapest_edge.at(neighbor) = el;
        }
      }
    }

    int key = 0;

    for (auto el : cheapest_edge) {
      if (el.from == -1) {
        continue;
      }
      spanning_tree.at(key) = el;
      key++;
    }

    for (const auto &el : spanning_tree) {
      res[el.from][el.to] = el.weight;
    }

    return res;
  }

  // 4
  TsmResult SolveTravelingSalesmanProblem(Graph &graph) {
    InitParameteres(graph);

    unsigned long int iteration = 0;
    while (iteration++ < it_сount_) {
      for (auto &ant : ants_) {
        bool have_way = true;

        while (ant.vertices.size() <= v_сount_ && have_way) {
          have_way = ChooseNextPoint(&ant, graph);
        }

        if ((ant.vertices[1]) && ant.vertices.size() == (v_сount_ + 1) &&
            ant.vertices.front() == ant.vertices.back()) {
          if (ant.distance < best_path_.distance) {
            best_path_ = ant;
            it_сount_ += v_сount_;
          }
        }
      }

      UpdatePheromone();

      for (auto &ant : ants_) {
        ant.distance = 0;
        ant.vertices.clear();
      }
    }

    for (unsigned long int i = 0; i < best_path_.vertices.size(); i++) {
      best_path_.vertices[i] += 1;
    }

    return best_path_;
  }

 private:
  unsigned long int v_сount_;
  unsigned long int it_сount_;
  unsigned long int ant_сount_;
  const double base_pheromone_ = 5;
  const double k_evaporatoin_ = 0.1;
  const double k_alpha_ = 1.2;
  const double k_beta_ = 1.0;
  const double k_Q_ = 400.0;

  std::vector<std::vector<double>> pheromone_;
  std::vector<TsmResult> ants_;
  TsmResult best_path_;

  bool HasElement(std::vector<int> visited, int elem) {
    for (unsigned long int i = 0; i < visited.size(); i++) {
      if (visited[i] == elem) return true;
    }
    return false;
  }

  void InitParameteres(Graph &graph) {
    v_сount_ = graph.GetSize();
    it_сount_ = v_сount_;
    ant_сount_ = v_сount_ * 2;

    std::vector<std::vector<double>> f_matrix(v_сount_,
                                              std::vector<double>(v_сount_));

    for (unsigned long int row = 0; row < v_сount_; row++) {
      for (unsigned long int col = 0; col < v_сount_; col++) {
        if (graph.GetMatrix()[row][col]) {
          f_matrix[row][col] = base_pheromone_;
        }
      }
    }

    pheromone_ = std::move(f_matrix);

    ants_.resize(ant_сount_);
    best_path_.distance = INT_MAX;
  }

  void UpdatePheromone() {
    for (unsigned long int i = 0; i < v_сount_; i++) {
      for (unsigned long int j = 0; j < v_сount_; j++) {
        pheromone_[i][j] *= (1 - k_evaporatoin_);
        if (pheromone_[i][j] < 0.1) pheromone_[i][j] = 0.1;
      }
    }

    for (unsigned long int i = 0; i < ant_сount_; i++) {
      if (ants_[i].distance != INT_MAX) {
        for (unsigned long int j = 0; j < v_сount_; j++) {
          int point1 = ants_[i].vertices[j];
          int point2 = ants_[i].vertices[j + 1];
          pheromone_[point1][point2] += (k_Q_ / ants_[i].distance);
        }
      }
    }
  }

  int RandomChoice() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 100);

    return dis(gen);
  }

  bool ChooseNextPoint(TsmResult *ant, Graph &graph) {
    int current_point;
    std::vector<int> variants;

    if (ant->vertices.empty()) {
      ant->vertices.push_back(0);
    }

    current_point = ant->vertices.back();

    for (unsigned long int i = 0; i < v_сount_; i++) {
      if ((graph.GetMatrix()[current_point][i]) &&
          !HasElement(ant->vertices, i))
        variants.push_back(i);
    }

    if (variants.empty()) {
      int first_element = ant->vertices[0];

      if (ant->vertices.size() == v_сount_ &&
          graph.GetMatrix()[current_point][first_element]) {
        ant->vertices.push_back(first_element);
        ant->distance += graph.GetMatrix()[current_point][first_element];
      } else {
        ant->distance = INT_MAX;
      }
      return false;
    }

    int nextPoint = GetNextPoint(variants, graph, current_point);
    ant->vertices.push_back(nextPoint);
    ant->distance += graph.GetMatrix()[current_point][nextPoint];

    return true;
  }

  std::vector<double> GetChance(std::vector<int> variants, Graph &graph,
                                int current_point) {
    std::vector<double> chance;
    std::vector<double> wish;
    double sum_wish = 0.0;

    for (auto v : variants) {
      double t = pheromone_[current_point][v];
      double n = 1.0 / graph.GetMatrix()[current_point][v];

      wish.push_back(pow(t, k_alpha_) * (pow(n, k_beta_)));
      sum_wish += wish.back();
    }

    for (auto w : wish) {
      chance.push_back(w / sum_wish * 100);
    }

    for (unsigned long int i = 1; i < chance.size(); i++) {
      chance[i] = chance[i] + chance[i - 1];
    }

    return chance;
  }

  int GetNextPoint(std::vector<int> variants, Graph &graph, int current_point) {
    int next_point;
    std::vector<double> chance = GetChance(variants, graph, current_point);
    double choose = RandomChoice();

    for (unsigned long int i = 0; i < variants.size(); i++) {
      if (choose <= chance[i]) {
        next_point = variants[i];
        break;
      }
    }

    return next_point;
  }
};

#endif  // _GRAPH_ALGORITHMS_H