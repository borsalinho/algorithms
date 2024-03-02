#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class Graph {
 public:
  Graph() : size_matrix_(0), is_loaded_(false){};
  ~Graph(){};

  void ExportGraphToDot(std::string filename) {
    if (!is_loaded_) throw "First you need to load the graph";

    std::ofstream output_file(filename + ".dot", std::ios::trunc);
    if (!output_file.is_open()) throw "Can't create file";

    output_file << ConvertToString();
    output_file.close();
  }

  void LoadGraphFromFile(std::string filename) {
    std::ifstream file(filename);

    if (!file.is_open()) throw "Can't find directory or open file";

    if (!ValidateFile(file)) {
      file.close();
      throw "Invalide file or incorrect data";
    }

    RemoveSpaces(file);
    file >> size_matrix_;
    if (size_matrix_ == 0) throw "The size can't be 0";

    matrix_.clear();
    matrix_.resize(size_matrix_, std::vector<int>(size_matrix_, 0));

    ReadElements(file);
    is_directed_ = IsDerected();
    is_loaded_ = true;
    file.close();
  }

  const std::string ToString() { return ConvertToString(); }

  std::vector<std::vector<int>> GetMatrix() { return matrix_; }

  int GetSize() { return size_matrix_; }

 private:
  std::vector<std::vector<int>> matrix_;
  size_t size_matrix_;
  bool is_directed_;
  bool is_loaded_;

  std::string ConvertToString() {
    if (size_matrix_ == 0) throw "Empty";

    std::ostringstream res;
    std::string padding = "  ";

    res << (is_directed_ ? "digraph" : "graph") << " {" << std::endl;

    for (size_t i = 0; i != size_matrix_; i++)
      res << padding << (i + 1) << ";" << std::endl;

    res << std::endl;
    std::string sep = is_directed_ ? "->" : "--";

    for (size_t i = 0; i != size_matrix_; i++) {
      for (size_t j = 0; j != size_matrix_; j++) {
        if (matrix_[i][j] != 0) {
          if (!is_directed_ && i > j) continue;

          std::string length =
              " [label=\"" + std::to_string(matrix_[i][j]) + "\"]";
          res << padding << (i + 1) << sep << (j + 1) << length << ";"
              << std::endl;
        }
      }
    }

    res << "}" << std::endl;

    return res.str();
    ;
  }

  bool IsDerected() {
    for (size_t i = 0; i != size_matrix_; i++) {
      for (size_t j = 0; j != size_matrix_; j++) {
        if (matrix_[j][i] != matrix_[i][j]) {
          return true;
        }
      }
    }
    return false;
  }

  void ReadElements(std::ifstream& file) {
    RemoveSpaces(file);

    for (size_t i = 0; i < size_matrix_; ++i) {
      std::vector<int> row;
      std::string line;
      std::getline(file, line);
      std::istringstream iss(line);
      int element;

      while (iss >> element) {
        row.push_back(element);
      }

      matrix_[i] = row;
    }
  }

  bool ValidateFile(std::ifstream& file) {
    char ch;

    while (file.get(ch)) {
      if (!std::isdigit(ch) && ch != ' ' && ch != '\n' && ch != '\t')
        return false;
    }

    file.clear();
    file.seekg(0, file.beg);

    return !file.fail() || file.eof();
  }

  void RemoveSpaces(std::ifstream& file) {
    while (file.peek() == ' ' || file.peek() == '\t' || file.peek() == '\n') {
      file.get();
    }
  }
};

#endif  //_GRAPH_H_