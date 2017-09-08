#ifndef CALLBACK_H
#define CALLBACK_H
#include "graph.h"

enum Color {white, grey, black};

int find_segments(Graph& graph, Graph_noprop& non_cuts);
int find_bad_cuts(Graph& graph, Graph_noprop& non_cuts);

class myGRBCallback: public GRBCallback
{
  public:
    Graph& graph;

    myGRBCallback(Graph& graph_) : graph(graph_) {};
  protected:
    void callback ();

};
#endif
