#include "graph.h"
#include <iostream>

using namespace std;
int main() {

  Graph *g = new Graph(6);

  g->addEdge(5, 0);
  g->addEdge(4, 0);
  g->addEdge(5, 2);
  g->addEdge(4, 1);
  g->addEdge(2, 3);
  g->addEdge(3, 1);

  g->printGraph();

  g->print_topological();
  g->print_khan();
}
