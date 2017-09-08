#include <iostream>
#include <string>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <vector>
#include <thread>

#include <boost/program_options.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "types_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/core/types_c.h"
#include "gurobi_c++.h"

#include "graph.h"
#include "callback.h"
   
#define DPRINT(X) std::cerr << #X << ": " << X << std::endl;
#define EPS 0.00001

namespace po = boost::program_options;
namespace b = boost;

	

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

cv::Mat src, src_gray;
cv::Mat dst;
cv::Mat detected_edges;
cv::Size picture_size;

//int edgeThresh = 1;
int lowThresh;
int const max_lowThresh = 100;
int ratio = 3; // 1:2 or 1:3
int kernel_size = 3;
const char* window_name = "Edge Map";

std::vector<GRBModel>* models_ref = NULL;
void my_handler(int s) {
  if(models_ref != NULL) {
    for(auto it = models_ref->begin(); it!=models_ref->end(); ++it) {
        it->terminate();
    }
    models_ref = NULL;
  }
  else {
    exit(2);
  }
}

void CannyThreshold(int, void*)
{
    /// Reduce noise with a kernel 3x3
    cv::blur( src_gray, detected_edges, cv::Size(3,3) );

    /// Canny detector
    cv::Canny( detected_edges, detected_edges, lowThresh, lowThresh*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    dst = cv::Scalar::all(0); // set all pixels of dst to greyscale color 0

    src.copyTo( dst, detected_edges); // copy with detected_edges as mask
    cv::imshow( window_name, dst );
}

int main(int argc, char** argv) {
    // set the handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = (void(*)(int))my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    //commandline
    po::variables_map vm;
    try {
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("grid-size,gs", po::value<int>()->default_value(1024), "set number of grid elements. Only approximate")
            ("input-file", po::value< std::string >(), "picture to process")
        ;

        po::positional_options_description p;
        p.add("input-file", -1);
        po::store(po::command_line_parser(argc, argv).
          options(desc).positional(p).run(), vm);
        //po::notify(vm);    
        if(vm.count("help")){
            std::cout << desc << std::endl;
            return EXIT_SUCCESS;
        }
        if(!vm.count("input-file")) {
            std::cerr << "no input file given" << std::endl;
            return EXIT_FAILURE;
        }
    }
    catch (po::error &ex) {
        std::cerr << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
    //parse image and build graph
    // Load an image
    #if 1 
    src = cv::imread( vm["input-file"].as<std::string>(), CV_LOAD_IMAGE_COLOR );

    if( !src.data )
    { 
        std::cerr << "bad input file" << std::endl;
        return EXIT_FAILURE; 
    }
    picture_size = src.size();

    dst.create( src.size(), src.type() );
    cv::cvtColor( src, src_gray, CV_BGR2GRAY );

    /// Create a window
    cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Create a Trackbar for user to enter threshold
    cv::createTrackbar( "Threshold:", window_name, &lowThresh, max_lowThresh, CannyThreshold );

    /// Show the image
    CannyThreshold(0, 0);
        
    cv::waitKey(0);

    unsigned concurentThreadsSupported = std::thread::hardware_concurrency();
    DPRINT(concurentThreadsSupported)
    //build submatrix per grid segment

    DPRINT(picture_size.width)
    DPRINT(picture_size.height)
    double ratio = ((double)picture_size.width)/picture_size.height;
    double root = sqrt(vm["grid-size"].as<int>());
    cv::Size segment_dim(floor(ratio * root), floor((1/ratio) * root));
    int number_of_segments = segment_dim.width*segment_dim.height;
    
    DPRINT(segment_dim.width)
    DPRINT(segment_dim.height)
    DPRINT(number_of_segments)
    //segments should like this:
    //D-A A A A A-C C C C
    //| | | | | | | | | |
    //B-X X X X-Y Y Y Y Y- ...
    //    S1       S2
    //B-X X X X-Y Y Y Y Y-
    //
    //B-X X X X-Y Y Y Y Y-
    // ...

    cv::Size2f segment_size_approx(picture_size.width / (float)segment_dim.width, picture_size.height / (float)segment_dim.height);
    DPRINT(segment_size_approx.width) 
    DPRINT(segment_size_approx.height) 

    
    double lambda = 0.09;
    std::vector<Graph*> segment_graphs(number_of_segments);
    //GRBEnv env = GRBEnv();
    std::vector<GRBEnv> envs(number_of_segments); // we need an environment per model
    std::vector<GRBModel> models;
    for(auto it = envs.begin(); it!=envs.end();++it) {
        auto env = *it;
        models.emplace_back(env);
    }
    models_ref = &models;

    int start_row = 0;
    int start_col = 0;

    for(int i = 0; i < segment_dim.height; ++i) {
        cv::Size segment_size;
        segment_size.height = (int)(floor(segment_size_approx.height*(i+1))-floor(segment_size_approx.height*i));
        if(i == segment_dim.height-1) segment_size.height = picture_size.height - start_row;
        
        for(int j = 0; j < segment_dim.width; ++j) {
            segment_size.width = (int)(floor(segment_size_approx.width*(j+1))-floor(segment_size_approx.width*j));
            if(j == segment_dim.width-1) segment_size.width = picture_size.width - start_col;

            int seg_num = xy_to_index(j, i, segment_dim);

            GRBModel& model = models[seg_num];
            // Turn off display and heuristics and enable adding constraints in our callback function
            model.set(GRB_IntParam_OutputFlag, 0); 
            model.set(GRB_DoubleParam_Heuristics, 1);
            model.set(GRB_IntParam_LazyConstraints, 1);
            
            segment_graphs[seg_num]= new Graph(segment_size.width*segment_size.height);
            Graph& g = *segment_graphs[seg_num];
            g[b::graph_bundle].size = segment_size;
            g[b::graph_bundle].row = cv::Range(start_row, start_row+segment_size.height);
            g[b::graph_bundle].col = cv::Range(start_col, start_col+segment_size.width);
            //build_grid
            GRBLinExpr obj1(0.0); // left side of term
            GRBLinExpr obj2(0.0); // right side of term
            GRBLinExpr objective(0.0);
            cv::Mat submatrix = src_gray(g[b::graph_bundle].row, g[b::graph_bundle].col);
            int edge_index_counter = 0;
            for(int y = 0; y < segment_size.height; ++y) {
                for(int x = 0; x < segment_size.width; ++x) {
                    if(x != segment_size.width-1) {
                        int a = xy_to_index(x, y, segment_size);
                        int b = xy_to_index(x+1, y, segment_size);
                        Graph::edge_descriptor e;
                        bool inserted;
                        b::tie(e, inserted) = b::add_edge(a, b, g);
                        g[e].index = edge_index_counter++;
                        g[e].var = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "edge");
                        //obj2 += lambda_row[y]*g[e].var;
                        obj2 += g[e].var;
                        //DPRINT(fabs(submatrix.at<uchar>(y,x)/255.f-submatrix.at<uchar>(y,x+1)/255.f))
                        obj1 += fabs(submatrix.at<uchar>(y,x)/255.f-submatrix.at<uchar>(y,x+1)/255.f)*g[e].var;
                    }
                    if(y != segment_size.height-1) {
                        int a = xy_to_index(x, y, segment_size);
                        int b = xy_to_index(x, y+1, segment_size);
                        Graph::edge_descriptor e;
                        bool inserted;
                        b::tie(e, inserted) = b::add_edge(a, b, g);
                        g[e].index = edge_index_counter++;
                        g[e].var = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "edge");
                        //obj2 += lambda_col[x]*g[e].var;
                        obj2 += g[e].var;
                        obj1 += sqrt(sqrt(fabs(submatrix.at<uchar>(y,x)/255.f-submatrix.at<uchar>(y+1,x)/255.f)))*g[e].var;
                    }
                }
            }
            //obj2 *= lambda;
            objective = obj2 - obj1;
            model.setObjective(objective, GRB_MINIMIZE);

            //canny edge constraints
            {
            //std::cout << type2str( detected_edges.type() ) << std::endl;
            cv::Mat submatrix = detected_edges(g[b::graph_bundle].row, g[b::graph_bundle].col);
            for(int y = 0; y < segment_size.height-1; ++y) {
                for(int x = 0; x < segment_size.width-1; ++x) {
                    //DPRINT((int)submatrix.at<uchar>(y,x))
                    if((int)submatrix.at<uchar>(y,x) == 255) {
                        //std::cout << "hi" << std::endl;
                        int a = xy_to_index(x, y, segment_size);
                        int b = xy_to_index(x+1, y, segment_size);
                        Graph::edge_descriptor e_horizontal, e_vertical;
                        bool exists;
                        b::tie(e_horizontal, exists) = b::edge(a, b, g);
                        a = xy_to_index(x, y, segment_size);
                        b = xy_to_index(x, y+1, segment_size);
                        b::tie(e_vertical, exists) = b::edge(a, b, g);
                        model.addConstr(g[e_horizontal].var + g[e_vertical].var >= 1);
                    }
                }
            }
            }
            //initial multicut constraints
            for(int x = 0; x < segment_size.width-1; ++x) {
                for(int y = 0; y < segment_size.height-1; ++y) {
                    //a---b
                    //|   |
                    //d---c
                    GRBVar& a_b = g[b::edge(xy_to_index(x, y, segment_size), xy_to_index(x+1,y, segment_size), g).first].var;
                    GRBVar& b_c = g[b::edge(xy_to_index(x+1, y, segment_size), xy_to_index(x+1,y+1, segment_size), g).first].var;
                    GRBVar& c_d = g[b::edge(xy_to_index(x, y+1, segment_size), xy_to_index(x+1,y+1, segment_size), g).first].var;
                    GRBVar& d_a = g[b::edge(xy_to_index(x, y+1, segment_size), xy_to_index(x,y, segment_size), g).first].var;
                    model.addConstr(a_b + b_c + c_d >= d_a);
                    model.addConstr(b_c + c_d + d_a >= a_b);
                    model.addConstr(c_d + d_a + a_b >= b_c);
                    model.addConstr(d_a + a_b + b_c >= c_d);
                }
            }
            #if 1
            myGRBCallback* cb = new myGRBCallback(g);
            model.setCallback(cb);
            #endif
            #if 1
            model.optimizeasync();
            #endif

            start_col += segment_size.width; 
        }
        start_col = 0;
        start_row += segment_size.height;
    }
    int modelnum = 0;
    while(modelnum < number_of_segments) {
        if(models[modelnum].get(GRB_IntAttr_Status) != GRB_INPROGRESS) {
            std::cout << "segment " << modelnum << " done" << std::endl;
            modelnum++;
        }
        sleep(0.5);
    }
    std::cout << "done. writing matrix" << std::endl;
    //WRITE
    cv::Mat red;
    red.create(picture_size, CV_8UC3);
    red = cv::Scalar(0, 0, 255);
    cv::Mat border_mask;
    border_mask.create(picture_size, CV_8UC1);
    border_mask = cv::Scalar(0); 

    for(int i = 0; i < number_of_segments; ++i){
        GRBModel& model = models[i];
        Graph& g = *segment_graphs[i];
        cv::Size& segment_size = g[b::graph_bundle].size;
        cv::Mat submatrix = border_mask(g[b::graph_bundle].row, g[b::graph_bundle].col);
        //segment border
        for(int x = 0; x < segment_size.width; ++x) {
            submatrix.at<uchar>(0, x) = 1;
            if(index_to_xy(i, segment_dim).y == segment_dim.height-1) submatrix.at<uchar>(segment_size.height-1, x) = 1;
        }
        for(int y = 0; y < segment_size.height; ++y) {
            submatrix.at<uchar>(y, 0) = 1;
            if(index_to_xy(i, segment_dim).x == segment_dim.width-1) submatrix.at<uchar>(y, segment_size.width-1) = 1;
        }
        #if 1
        //horizontal edges
        for(int x = 0; x < (segment_size.width-1); ++x) {
          for(int y = 0; y < segment_size.height; ++y) {
            if(std::abs(g[b::edge(xy_to_index(x, y, segment_size), xy_to_index(x+1,y,segment_size), g).first].var.get(GRB_DoubleAttr_X) - 1.0) < EPS) {
                submatrix.at<uchar>(y, x) = 1; // left
            }
          }
        }

        //vertical edges
        for(int x = 0; x < segment_size.width; ++x) {
          for(int y = 0; y < (segment_size.height-1); ++y) {
            if(std::abs(g[b::edge(xy_to_index(x, y, segment_size), xy_to_index(x,y+1,segment_size), g).first].var.get(GRB_DoubleAttr_X) - 1.0) < EPS) {
                submatrix.at<uchar>(y+1, x) = 1; // down 
            }
          }
        }
        #endif
    }
    std::cout << "showing image" << std::endl;
    cv::cvtColor(src_gray, dst, CV_GRAY2BGR);
    red.copyTo( dst, border_mask ); 
    cv::imshow( window_name, dst );

    #if 0
    Graph picture_graph(picture_size.width*picture_size.height);
    int edge_index_counter = 0;
    for(int x = 0; x < picture_size.width; ++x) {
        for(int y = 0; y < picture_size.height; ++y) {
            if(x != picture_size.width-1) {
                int a = xy_to_index(x, y, picture_size);
                int b = xy_to_index(x+1, y, picture_size);
                Graph::edge_descriptor e;
                bool inserted;
                b::tie(e, inserted) = b::add_edge(a, b, picture_graph);
                picture_graph[e].index = edge_index_counter++;
            }
            if(y != picture_size.height-1) {
                int a = xy_to_index(x, y, picture_size);
                int b = xy_to_index(x, y+1, picture_size);
                Graph::edge_descriptor e;
                bool inserted;
                b::tie(e, inserted) = b::add_edge(a, b, picture_graph);
                picture_graph[e].index = edge_index_counter++;
            }
        }
    }
    #endif
    
    cv::waitKey(0);

    std::cout << "...DONE." << std::endl;
        
    


    #endif
    return EXIT_SUCCESS;
}
