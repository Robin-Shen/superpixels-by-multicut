#ifndef GRAPH_H
#define GRAPH_H

#include <boost/utility.hpp>                // for boost::tie
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/copy.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "gurobi_c++.h"
//#include "math_vector.h"

struct EdgeProperties{
    GRBVar var;
    int index;
    bool error;
};

struct VertexProperties{
    bool canny_edge;
    int multicut_label;
};

struct GraphProperties {
    cv::Range row;
    cv::Range col;
    cv::Size size;
};

struct do_nothing
{
  template <typename VertexOrEdge1, typename VertexOrEdge2>
  void operator()(const VertexOrEdge1& , VertexOrEdge2& ) const
  {
  }
}; // for copying

// create a typedef for the Graph type
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS> Graph_noprop; // used for finding bad Cuts
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties, GraphProperties> Graph;


template <typename VertexIterator>
VertexIterator xy_to_iterator(int x, int y, VertexIterator v, cv::Size size) {
  assert(abs(x) <= size.width && abs(y) <= size.height);
  v += x + y*size.width;
  return v;
};

inline int xy_to_index(int x, int y, cv::Size size) {
  assert(abs(x) <= size.width && abs(y) <= size.height);
  return x + y*size.width;
};

inline cv::Point2i index_to_xy(int index, cv::Size size) {
  int x = index % size.width;
  int y = index / size.width;
  return cv::Point2i(x, y);
};
//xEdges and yEdges indices functions
//inline int x_index(int x, int y) {return x+(size.width-1)*y;}
//inline int y_index(int x, int y) {return y+(size.height-1)*x;}


#endif
