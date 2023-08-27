#pragma once

#include <algorithm>
#include <iostream>
#include <stack>
#include <utility>
#include <vector>

class Graph {
private:
  // structures
  int v;
  std::vector<std::vector<int>> adj;
  // used for kosaraju
  void getFinishTime(int curr, std::vector<int> &visited,
                     std::stack<int> &finish_time);

  void print_dfs(int curr, std::vector<int> &vis);

  // for bridges
  void dfs_bridge(int node, int parent, int low[], int time[],
                  std::vector<int> &vis,
                  std::vector<std::pair<int, int>> &bridges, int &timer);

public:
  Graph(int noNodes);

  // utils
  int addEdge(int a, int b);
  bool isPath(int src, int des);
  Graph *getTransporse();

  // visualize
  void printGraph();

  // traversal

  bool dfs_until(int curr, int des, std::vector<int> &vis);
  void dfs(int curr, std::vector<int> &vis);

  // algorithms
  std::vector<std::vector<int>> findScc();
  // kosaraju
  bool isScK();
  void print_ScK();
  // targans
  void print_bridges();
};
