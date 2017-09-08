#include "callback.h"
#include <queue>

namespace b = boost;

#define EPS 0.000001

// for adding constrains lazily
// we use the gurobi callback mechanism
void myGRBCallback::callback() {
  try {
    if (where == GRB_CB_MIPSOL) {
      // MIP solution callback
      int nodecnt = (int) getDoubleInfo(GRB_CB_MIPSOL_NODCNT); // node number
      double obj = getDoubleInfo(GRB_CB_MIPSOL_OBJ); // objective value
      double obj_bnd = getDoubleInfo(GRB_CB_MIPSOL_OBJBND); // current error
      int solcnt = getIntInfo(GRB_CB_MIPSOL_SOLCNT); // solution number

      // build new graph but only with edges, if there is no cutting edge in the graph (so if sol = 0)
      Graph_noprop non_cuts(b::num_vertices(graph));
      Graph::edge_iterator ei, ei_end;
      for(b::tie(ei, ei_end) = b::edges(graph); ei != ei_end; ++ei) {
        graph[*ei].error = false; // (re)set to false
        // if(getSolution(graph[*ei].var)!=0.0 && getSolution(graph[*ei].var)!=1.0)
        //   std::cout << getSolution(graph[*ei].var) << std::endl;
        if(std::abs(getSolution(graph[*ei].var) - 0.0) < EPS) {
          Graph_noprop::vertex_descriptor src = source(*ei, graph), targ = target(*ei, graph);
          add_edge(src, targ, non_cuts);
        }
      }

      find_segments(graph, non_cuts);
      find_bad_cuts(graph, non_cuts);

      for(b::tie(ei, ei_end) = b::edges(graph); ei != ei_end; ++ei) {
        if(!graph[*ei].error)
          continue;
        Graph::vertex_descriptor u = source(*ei, graph), v = target(*ei, graph);

        // find shortest path from u to v
        std::vector<Graph_noprop::vertex_descriptor> p(b::num_vertices(graph), -1);
        std::vector<Color> color(b::num_vertices(graph), white);
        std::queue<Graph_noprop::vertex_descriptor> Q;
        p[u] = -1;
        color[u] = grey;
        Q.push(u);
        while(!Q.empty()) {
          Graph_noprop::vertex_descriptor u = Q.front();
          Q.pop();
          Graph_noprop::adjacency_iterator ai, ai_end;
          for(b::tie(ai, ai_end) = b::adjacent_vertices(u, non_cuts); ai != ai_end; ++ai){
            if(color[*ai] == white){
              color[*ai] = grey;
              p[*ai] = u;
              if(*ai == v) {
                goto found_it;
              }
              Q.push(*ai);
            }
          }
          color[u] = black;
        }
        found_it:
        GRBLinExpr sum_u_to_v(0.0);
        Graph_noprop::vertex_descriptor a1 = v;
        Graph_noprop::vertex_descriptor a2 = p[v];
        while(a2 != -1) {
          Graph::edge_descriptor e_path = b::edge(a1,a2,graph).first;
          sum_u_to_v += graph[e_path].var;
          a1 = a2;
          a2 = p[a1];
        }
        addLazy(sum_u_to_v >= graph[*ei].var);
      }
    }
  } catch (GRBException e) {
    std::cout << "Error number: " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Error during callback" << std::endl;
  }
}


int find_segments(Graph& graph, Graph_noprop& non_cuts) {
  std::vector<Graph_noprop::vertex_descriptor> p(b::num_vertices(graph), -1);
  std::vector<Color> color(b::num_vertices(graph), white);
  std::queue<Graph_noprop::vertex_descriptor> Q;
  int segment_count = 0;
  {
    Graph::vertex_iterator vi, vi_end;
    b::tie(vi, vi_end) = b::vertices(graph);
    for(auto it = vi; it != vi_end; ++it) {
      graph[*it].multicut_label = -1;
    }
  }
  Graph_noprop::vertex_iterator vi, vi_end;
  b::tie(vi, vi_end) = b::vertices(non_cuts);
  for(auto it = vi; it != vi_end; ++it) {
    Graph_noprop::vertex_descriptor s = *it;
    if(graph[s].multicut_label != -1)
      continue;

    segment_count++;
    p[s] = -1;
    graph[s].multicut_label = segment_count;
    color[s] = grey;
    Q.push(s);
    while(!Q.empty()) {
      Graph_noprop::vertex_descriptor u = Q.front();
      Q.pop();
      Graph_noprop::adjacency_iterator ai, ai_end;
      for(b::tie(ai, ai_end) = b::adjacent_vertices(u, non_cuts); ai != ai_end; ++ai){
        if(color[*ai] == white){
          color[*ai] = grey;
          p[*ai] = u;
          graph[*ai].multicut_label = segment_count;
          Q.push(*ai);
        }
      }
      color[u] = black;
    }
  }

  return segment_count;
}

int find_bad_cuts(Graph& graph, Graph_noprop& non_cuts){
  int num_bad_cuts = 0;
  Graph::edge_iterator ei, ei_end;
  for(b::tie(ei, ei_end) = b::edges(graph); ei != ei_end; ++ei) {
    Graph_noprop::vertex_descriptor src = source(*ei, graph), targ = target(*ei, graph);
    bool cut = !b::edge(src, targ, non_cuts).second;
    if(cut && graph[src].multicut_label == graph[targ].multicut_label && !graph[*ei].error) { // cut in same segment!
      graph[*ei].error = true; // we don't want to find the same bad cut twice
      num_bad_cuts++;
      #ifdef DEBUG
      auto u_pos = index_to_xy(u);
      auto v_pos = index_to_xy(v);
      std::cout << "found bad cut: " << "(" << u_pos.x << ", " << u_pos.y << ")" << " to " <<
      "(" << v_pos.x << ", " << v_pos.y << ")" << std::endl;
      #endif
    }
  }
  return num_bad_cuts;
}

#undef DEBUG
