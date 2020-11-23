#pragma once
#include <cstddef>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <stdexcept>
#include <stdio.h>
/*
 * Description:     A path planning class implementation for the project
 *                  RCA 5.semester.
 *
 * Developer:
 * Creation date:   281020
 *
 * Changelog:       DDMMYY  XX  Change
 *                  281020  TH  Started construction of the path planning class
 *              pre 111120  TH  Implemented the quasi-random node selection
 *                              algorithm and started a node validation
 *                              algorithm
 *                  111120  TH  Removed a bug in get_pixel_color function that
 *                              returns invalid colors
 *                  111120  TH  Implemented the node validation algorithm part
 *                              that checks if a node is inside a wall
 *                  111120  TH  Implemented the path_node class
 *                  111120  TH  Implemented the node path-planning algorithms
 *                  121120  TH  Fixed bug where the hammersley algorithm
 *                              sometimes returned negative nodes, and fixed a
 *                              bug where non-random patterns appeared at larger
 *                              node sets
 *                  121120  TH  Fixed some setup stuff and tried out integer
 *                              return instead of void for error returns
 *
 *
 * NOTES:
 * A Hammersley or Halton sequence might be good for a sampling strategy.
 * Algorithms such as RRT (Rapidly-exploring Random Tree) with a merge might be
 * good but also complicated (found on page ~240)
 * https://www.researchgate.net/publication/244441430_Sampling_with_Hammersley_and_Halton_Points
 *
 * https://www.rajgunesh.com/resources/downloads/statistics/samplingmethods.pdf
 * Page 203 (chapter 7.1.1) in the book from Robots in Context
 * Page 220 --||--
 * 1. Random waypoint assignment algorithm
 * 2. Invalid waypoint elimination
 * 3. Density waypoint elimination
 * 4. waypoint path connection
 * 5. path density elimination
 * return waypoint_node vector and image with drawn on waypoints and paths.
 *
 *------------------------------------------------------------------------------
 * brushfire først så ingen noder er for tæt på kanterne
 * brushfire -> super node reduktion
 */

enum DODEBUG { NODEBUG = 0, DEBUG = 1 };
enum DOWAIT { NOWAIT = 0, WAIT = 1 };
enum WHATCOLOR { GREYSCALE = 0, COLOR = 1 };
#define GREEN                                                                  \
  { 0, 255, 0 }
#define BLUE                                                                   \
  { 255, 0, 0 }
#define RED                                                                    \
  { 0, 0, 255 }
#define WHITE                                                                  \
  { 255, 255, 255 }
#define BLACK                                                                  \
  { 0, 0, 0 }

const cv::Vec3b green_pixel = GREEN;
const cv::Vec3b blue_pixel = BLUE;
const cv::Vec3b red_pixel = RED;
const cv::Vec3b white_pixel = WHITE;
const cv::Vec3b black_pixel = BLACK;

////////////////////////////////////////////////////////////////////////////////
// PATH NODE CLASS
////////////////////////////////////////////////////////////////////////////////
class path_node {
private:
  std::vector<path_node> connector_nodes;
  cv::Point this_position;

public:
  //----------------------------------------------------------------------------
  // CONSTRUCTOR
  //----------------------------------------------------------------------------
  path_node(cv::Point _position) : this_position(_position){};

  //----------------------------------------------------------------------------
  // GET POSITION OF NODE
  //----------------------------------------------------------------------------
  cv::Point get_position(void) { return this_position; }

  //----------------------------------------------------------------------------
  // GET CONNECTOR NODES
  //----------------------------------------------------------------------------
  std::vector<path_node> get_possible_paths(void) { return connector_nodes; }

  //----------------------------------------------------------------------------
  // ADD CONNECTOR NODE
  //----------------------------------------------------------------------------
  void add_possible_path(path_node _node) { connector_nodes.push_back(_node); }

  //----------------------------------------------------------------------------
  // PRINT ALL DATA INCLUDING CONNECTOR NODE POSITIONS
  //----------------------------------------------------------------------------
  void print_node_data(void) {
    std::cout << "node position: " << this_position << ", "
              << connector_nodes.size() << " node connections: " << std::endl;
    for (size_t i = 0; i < connector_nodes.size(); i++) {
      std::cout << connector_nodes[i].get_position() << std::endl;
    }
  }
};

////////////////////////////////////////////////////////////////////////////////
// PATH PLANNING CLASS
////////////////////////////////////////////////////////////////////////////////
class brushfire_pixel {
public:
  cv::Point position;
  int generation_index;
  //----------------------------------------------------------------------------
  // CONSTRUCTOR
  //----------------------------------------------------------------------------
  brushfire_pixel(cv::Point _position, int _generation_index)
      : position(_position), generation_index(_generation_index){};
};
////////////////////////////////////////////////////////////////////////////////
// PATH PLANNING CLASS
////////////////////////////////////////////////////////////////////////////////
class path_planning {
private:
  cv::Mat input_map;
  cv::Mat node_map;
  std::string map_name;
  std::vector<cv::Point> waypoint_nodes;
  std::vector<path_node> waypoint_path_nodes;
  int map_width;
  int map_height;

  //----------------------------------------------------------------------------
  // GENERATE RANDOM POINTS
  //----------------------------------------------------------------------------
  cv::Point generate_random_point(bool _debug = NODEBUG) {
    cv::Point result = {(rand() % map_width), (rand() % map_height + 1)};
    if (_debug) {
      std::cout << "Generated point: " << result << std::endl;
    }
    return result;
  }

  //----------------------------------------------------------------------------
  // GET PIXEL VALUE ARRANGED IN [BLUE GREEN RED] ORDER
  //----------------------------------------------------------------------------
  cv::Vec3b get_pixel_color(cv::Point _point) {
    /*white = [255, 255, 255], black = [0, 0, 0]*/
    return node_map.at<cv::Vec3b>(_point);
  }

  //----------------------------------------------------------------------------
  // CHECK IF A POSITION IS VALID INSIDE MAP
  //----------------------------------------------------------------------------
  bool on_board(cv::Point next_position) {
    if (next_position.x < 0 || next_position.x > map_width ||
        next_position.y < 0 || next_position.y > map_height) {
      return true;
    }
    return false;
  }

public:
  //----------------------------------------------------------------------------
  // CONSTRUCTOR
  //----------------------------------------------------------------------------
  path_planning(cv::Mat _input_map, std::string _name = "")
      : input_map(_input_map), node_map(_input_map), map_name(_name),
        map_width(_input_map.cols), map_height(_input_map.rows){};

  //----------------------------------------------------------------------------
  // SETUP RANDOM GENERATOR
  //----------------------------------------------------------------------------
  void random_generator_setup() { srand(time(NULL)); }

  //----------------------------------------------------------------------------
  // SHOW THE NODE MAP
  //----------------------------------------------------------------------------
  int show_map(float _scale_size = 1, bool _wait = NOWAIT) {
    if (node_map.empty()) {
      return 0;
    }
    cv::Mat view_map = node_map;
    cv::resize(view_map, view_map, cv::Size(), _scale_size, _scale_size,
               cv::INTER_NEAREST);
    cv::namedWindow(map_name);
    cv::imshow(map_name, view_map);
    if (_wait) {
      cv::waitKey();
    }
    return 1;
  }

  //----------------------------------------------------------------------------
  // RESIZE THE IN MEMORY MAP
  //----------------------------------------------------------------------------
  void resize_map(int _scale_size) {
    cv::resize(node_map, node_map, cv::Size(), _scale_size, _scale_size,
               cv::INTER_NEAREST);
    map_width = node_map.cols;
    map_height = node_map.rows;
  }

  //----------------------------------------------------------------------------
  // GENERATE QUASIRANDOM HAMMERSLEY NODES AND SAVE
  //----------------------------------------------------------------------------
  void generate_quasirandom_hammersley_nodes(int _node_number) {
    const int max_map_width = 1;
    const float half_map_width = 0.5;
    float displacement;
    float width_position, height_position;
    int current_node, current_node_expansion;

    /*generate n nodes for the map*/
    for (current_node = 0; current_node <= _node_number; current_node++) {
      width_position = 0;
      displacement = max_map_width;

      /*calculate the displacement of the node*/
      for (current_node_expansion = current_node;
           displacement *= half_map_width; current_node_expansion >>= 1) {

        /*check if current_node_expansion is odd*/
        if (current_node_expansion & 1) {
          /*calculate the width and height of the current node*/
          width_position += displacement;
          height_position = (current_node + half_map_width) / _node_number;
        }
      }
      /*upscale calculated waypoint to fit node_map and place*/
      if (width_position > 0 && height_position > 0) {
        waypoint_nodes.push_back({int(width_position * map_width),
                                  int(height_position * map_height)});
      }
    }
  }

  //----------------------------------------------------------------------------
  // PAD WALLS WITH A BRUSHFIRE LIKE ALGORITHM
  //----------------------------------------------------------------------------
  void pad_walls(int _padding_pixels = 1, int _debug = NODEBUG) {
    if (_debug) {
      std::cout << "PAD WALLS STARTED" << std::endl;
    }
    /*The possible moves that the padding algorithm needs to take*/
    std::vector<cv::Point> movement_grid = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    std::queue<brushfire_pixel *> brushfire_queue;

    /*Initialization*/
    for (int i = 0; i < map_width; i++) {
      for (int i = 0; i < map_height; i++) {
        if (_debug) {
          std::cout << "current pixel color: "
                    << get_pixel_color({map_width, map_height}) << std::endl;
        }
        if (get_pixel_color({map_width, map_height}) == black_pixel) {
          if (_debug) {
            std::cout << "pushed back pixel at: " << map_width << " "
                      << map_height << std::endl;
          }
          brushfire_queue.push(new brushfire_pixel({map_width, map_height}, 0));
        }
      }
    }
    /*Guard clause*/
    if (brushfire_queue.empty()) {
      return;
    }
    /*Brushfire*/
    brushfire_pixel *current_pixel;
    while (!brushfire_queue.empty()) {
      current_pixel = brushfire_queue.front();
      brushfire_queue.pop();
      if (current_pixel->generation_index >= _padding_pixels) {
        break;
      }
      for (size_t i = 0; i < movement_grid.size(); i++) {
        cv::Point next_position = {
            current_pixel->position.x + movement_grid[i].x,
            current_pixel->position.y + movement_grid[i].y};
        /*Valid position*/
        if (on_board(next_position)) {
          if (get_pixel_color(next_position) == black_pixel) {
            cv::line(node_map, next_position, next_position, black_pixel);
            brushfire_queue.push(new brushfire_pixel(
                next_position, current_pixel->generation_index + 1));
          }
        }
      }
    }
  }

  //----------------------------------------------------------------------------
  // REMOVE UNWANTED AND INVALID NODES FROM THE NODE LIST
  //----------------------------------------------------------------------------
  void remove_unwanted_nodes(bool _debug = NODEBUG) {
    std::vector<cv::Point> valid_waypoint_nodes = {};
    for (size_t i = 0; i < waypoint_nodes.size(); i++) {

      if (get_pixel_color(waypoint_nodes[i]) != black_pixel) {
        valid_waypoint_nodes.push_back(waypoint_nodes[i]);
      } else {
        if (_debug) {
          std::cout << "removed node: " << i << " " << waypoint_nodes[i]
                    << std::endl;
        }
      }
    }
    waypoint_nodes = valid_waypoint_nodes;
  }

  //----------------------------------------------------------------------------
  // PRINT WAYPOINT NODE LIST
  //----------------------------------------------------------------------------
  void print_waypoint_nodes(void) {
    std::cout << waypoint_nodes.size() << " nodes:" << std::endl;
    for (size_t i = 0; i < waypoint_nodes.size(); i++) {
      std::cout << waypoint_nodes[i] << " ";
    }
    std::cout << std::endl;
  }

  //----------------------------------------------------------------------------
  // COLOR WAYPOINT NODE LIST ONTO NODE MAP
  //----------------------------------------------------------------------------
  void color_waypoint_nodes(cv::Vec3b _color = green_pixel) {
    for (size_t i = 0; i < waypoint_nodes.size(); i++) {
      cv::line(node_map, waypoint_nodes[i], waypoint_nodes[i], _color);
    }
  }

  //----------------------------------------------------------------------------
  // FIND IF A BIRD PATH IS POSSIBLE
  //----------------------------------------------------------------------------
  int valid_bird_path(cv::Point _start, cv::Point _goal,
                      bool _debug = NODEBUG) {

    if (_start == _goal) {
      return -1; /*Disaster*/
    }
    cv::LineIterator bird_path(node_map, _start, _goal, 8);

    if (_debug) {
      std::cout << "amount of pixels in line: " << bird_path.count << std::endl;
    }
    for (int i = 0; i < bird_path.count; i++, ++bird_path) {
      if (cv::Vec3b(*bird_path) == black_pixel) {
        return 0; /*failure*/
      } else {
        if (_debug) {
          std::cout << i << " " << bird_path.pos() << std::endl
                    << " " << get_pixel_color(bird_path.pos()) << std::endl;
        }
      }
    }
    return 1; /*Success*/
  }

  //----------------------------------------------------------------------------
  // FIND CONNECTIONS BETWEEN NODES = O(N^2) TIME COMPLEXITY
  //----------------------------------------------------------------------------
  void find_node_map_connections(void) {
    for (size_t i = 0; i < waypoint_nodes.size(); i++) {
      path_node i_current_node(waypoint_nodes[i]);
      for (size_t j = 0; j < waypoint_nodes.size(); j++) {
        path_node j_current_node(waypoint_nodes[j]);
        if (valid_bird_path(waypoint_nodes[i], waypoint_nodes[j]) &&
            i_current_node.get_position() != j_current_node.get_position()) {

          i_current_node.add_possible_path(j_current_node);
        }
      }
      waypoint_path_nodes.push_back(i_current_node);
    }
  }

  //----------------------------------------------------------------------------
  // PRINT CONNECTIONS BETWEEN NODES
  //----------------------------------------------------------------------------
  void print_node_map_connections(void) {
    for (size_t i = 0; i < waypoint_path_nodes.size(); i++) {
      waypoint_path_nodes[i].print_node_data();
    }
  }

  //----------------------------------------------------------------------------
  // DRAW CONNECTIONS BETWEEN NODES = O(N^2) TIME COMPLEXITY
  //----------------------------------------------------------------------------
  void draw_node_map_connections(cv::Vec3b _color = green_pixel) {
    for (size_t i = 0; i < waypoint_path_nodes.size(); i++) {

      size_t connections = waypoint_path_nodes[i].get_possible_paths().size();
      for (size_t j = 0; j < connections; j++) {
        cv::line(node_map, waypoint_path_nodes[i].get_position(),
                 waypoint_path_nodes[i].get_possible_paths()[j].get_position(),
                 _color);
      }
    }
  }
};
