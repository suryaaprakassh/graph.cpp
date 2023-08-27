#include "graph.h"
Graph::Graph(int noNodes) {
  v = noNodes;
  for (int i = 0; i < v; i++) {
    adj.push_back(std::vector<int>());
  }
}

int Graph::addEdge(int a, int b) {

  if (a >= 0 && a < v && b >= 0 && b < v) {
    adj[a].push_back(b);
    return 1;
  } else {
    std::cout << "Warning: Node exceeds graph size!" << std::endl;
    std::cout << "Node (" << a << "," << b << ") Not Inserted!" << std::endl;
    return 0;
  }
}

void Graph::printGraph() {
  for (int i = 0; i < v; i++) {
    std::cout << i << " -> ";
    for (auto a : adj[i]) {
      std::cout << a << " -> ";
    }
    std::cout << "end" << std::endl;
  }
}

bool Graph::dfs_until(int curr, int des, std::vector<int> &vis) {
  if (curr == des)
    return true;
  vis[curr] = true;
  for (auto x : adj[curr]) {
    if (!vis[x]) {
      if (dfs_until(x, des, vis)) {
        return true;
      }
    }
  }
  return false;
}

void Graph::dfs(int curr, std::vector<int> &vis) {
  vis[curr] = true;
  for (auto x : adj[curr]) {
    if (!vis[x]) {
      dfs(x, vis);
    }
  }
}

void Graph::print_dfs(int curr, std::vector<int> &vis) {
  vis[curr] = true;
  std::cout << curr << " ";
  for (auto x : adj[curr]) {
    if (!vis[x]) {
      print_dfs(x, vis);
    }
  }
}
void Graph::dfs_bridge(int node, int parent, int low[], int time[],
                       std::vector<int> &vis,
                       std::vector<std::pair<int, int>> &bridges, int &timer) {
  vis[node] = 1;
  time[node] = low[node] = timer;
  timer++;
  for (auto it : adj[node]) {
    if (it == parent)
      continue;
    if (!vis[it]) {
      dfs_bridge(it, node, low, time, vis, bridges, timer);
      low[node] = std::min(low[node], low[it]);
      if (low[it] > time[node]) {
        bridges.push_back({it, node});
      }
    } else {
      low[node] = std::min(low[node], low[it]);
    }
  }
}

void Graph::getFinishTime(int curr, std::vector<int> &visited,
                          std::stack<int> &finish_time) {
  visited[curr] = true;
  for (auto x : adj[curr]) {
    if (!visited[x]) {
      visited[x] = 1;
      getFinishTime(x, visited, finish_time);
    }
  }
  finish_time.push(curr);
}

bool Graph::isPath(int src, int des) {
  std::vector<int> vis(v, 0);
  return dfs_until(src, des, vis);
}

// brute force
std::vector<std::vector<int>> Graph::findScc() {

  std::vector<std::vector<int>> ans;

  std::vector<int> is_scc(v, 0);

  for (int i = 0; i < v; i++) {
    if (!is_scc[i]) {
      is_scc[i] = 1;
      std::vector<int> curr_scc;
      curr_scc.push_back(i);

      for (int j = i + 1; j <= v; j++) {
        if (!is_scc[j] && isPath(i, j) && isPath(j, i)) {
          is_scc[j] = 1;
          curr_scc.push_back(j);
        }
      }

      ans.push_back(curr_scc);
    }
  }
  return ans;
}

Graph *Graph::getTransporse() {
  Graph *trans = new Graph(v);

  for (int i = 0; i < v; i++) {
    for (auto ad : adj[i]) {
      trans->addEdge(ad, i);
    }
  }

  return trans;
}

bool Graph::isScK() {
  std::vector<int> visited(v, 0);
  dfs(0, visited);
  for (int i : visited) {
    if (!i) {
      return 0;
    }
  }

  Graph *trans = getTransporse();

  std::fill(visited.begin(), visited.end(), 0);

  trans->dfs(0, visited);

  for (int i : visited) {
    if (!i) {
      return 0;
    }
  }

  return 1;
}

void Graph::print_ScK() {

  std::stack<int> finish_time;
  std::vector<int> visited(v, 0);
  for (int i = 0; i < v; i++) {
    if (!visited[i]) {
      getFinishTime(i, visited, finish_time);
    }
  }
  Graph *trans = getTransporse();
  std::fill(visited.begin(), visited.end(), 0);
  int c = 0;
  std::cout << "Strongly Connected Components:\n";
  while (!finish_time.empty()) {
    int node = finish_time.top();
    finish_time.pop();
    if (!visited[node]) {
      c += 1;
      trans->print_dfs(node, visited);
      std::cout << "\n";
    }
  }
  std::cout << "The number of Strongly connected Components:" << c << "\n";
}

void Graph::print_bridges() {
  std::vector<int> vis(v, 0);
  int low[v];
  int time[v];
  std::vector<std::pair<int, int>> bridges;
  int timer = 1;

  for (int i = 0; i < v; i++) {
    dfs_bridge(i, -1, low, time, vis, bridges, timer);
  }
  std::cout << "Briges:"
            << "\n";
  for (auto i : bridges) {
    std::cout << "(" << i.first << "," << i.second << ")"
              << "\n";
  }
}
