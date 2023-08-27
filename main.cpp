#include "graph.h"
#include <iostream>

using namespace std;
int main() {

  Graph *g = new Graph(2);

  g->addEdge(0, 1);
  g->addEdge(1, 0);

  g->print_bridges();

  return 0;
}
