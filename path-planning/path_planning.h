#pragma once
#include <cstddef>
#include <iostream>
#include <opencv2/core/matx.hpp>
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
 *                  231120  TH/ER   Started brushfire padding algorithm
 *                  261120  TH/ER   Finished the A* pathfinding algorithm
 *                  271120  TH/ER   Recommented A* code and made some functions
 *                                  private. Fixed bug that didnt allow code to
 *                                  execute without using cv::imshow() first
 *                                  Not really a bug "fix" but a workaround.
 *
 */

enum DODEBUG { NO_DEBUG = 0, NODEBUG = 0, DEBUG = 1 };
enum DOWAIT { NO_WAIT = 0, NOWAIT = 0, WAIT = 1 };
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
#define GREY                                                                   \
  { 169, 169, 169 }

const cv::Vec3b green_pixel = GREEN;
const cv::Vec3b blue_pixel = BLUE;
const cv::Vec3b red_pixel = RED;
const cv::Vec3b white_pixel = WHITE;
const cv::Vec3b black_pixel = BLACK;
const cv::Vec3b grey_pixel = GREY;

////////////////////////////////////////////////////////////////////////////////
// PATH NODE CLASS
////////////////////////////////////////////////////////////////////////////////
class path_node {
private:
  std::vector<cv::Point> connector_nodes;
  cv::Point this_position;

public:
  /*costs for path_finding*/
  float g_cost = 0.0f;
  float h_cost = 0.0f;
  float f_cost = 0.0f;
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
  std::vector<cv::Point> get_possible_paths(void) { return connector_nodes; }

  //----------------------------------------------------------------------------
  // ADD CONNECTOR NODE
  //----------------------------------------------------------------------------
  void add_possible_path(cv::Point _node) { connector_nodes.push_back(_node); }

  //----------------------------------------------------------------------------
  // PRINT ALL DATA INCLUDING CONNECTOR NODE POSITIONS
  //----------------------------------------------------------------------------
  void print_node_data(void) {
    std::cout << "node position: " << this_position << ", "
              << connector_nodes.size() << " node connections: " << std::endl;
    for (size_t i = 0; i < connector_nodes.size(); i++) {
      std::cout << connector_nodes[i] << std::endl;
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
  cv::Mat eroded_map;

  std::string map_name;
  std::vector<cv::Point> waypoint_nodes;
  std::vector<path_node> waypoint_path_nodes;
  std::vector<path_node> room_nodes;
  int scaling_factor = 1;

  //----------------------------------------------------------------------------
  // GENERATE RANDOM POINTS
  //----------------------------------------------------------------------------
  cv::Point generate_random_point(bool _debug = NO_DEBUG) {
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
  // GET PIXEL VALUE OF HE ERODED MAP ARRANGED IN [BLUE GREEN RED] ORDER
  //----------------------------------------------------------------------------
  cv::Vec3b get_eroded_pixel_color(cv::Point _point) {
    /*white = [255, 255, 255], black = [0, 0, 0]*/
    return eroded_map.at<cv::Vec3b>(_point);
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

  //----------------------------------------------------------------------------
  // GET NODE FROM COORDINATE
  //----------------------------------------------------------------------------
  path_node get_node_from_coordinate(cv::Point _coordinate,
                                     int _debug = NO_DEBUG) {
    for (size_t i = 0; i < waypoint_path_nodes.size(); i++) {
      if (waypoint_path_nodes[i].get_position() == _coordinate) {
        if (_debug) {
          std::cout << "returns node at: "
                    << waypoint_path_nodes[i].get_position() << std::endl;
        }
        return waypoint_path_nodes[i];
      }
    }
    if (_debug) {
      std::cout << "NO NODE FOUND -> RETURNS INVALID NODE" << std::endl;
    }
    return path_node({-1, -1});
  }

  //----------------------------------------------------------------------------
  // HEURISTIC DISTANCE BETWEEN 2 NODES
  //----------------------------------------------------------------------------
  float heuristic(path_node _a, path_node _b) {
    /*Manhatten Distance*/
    return std::abs((_a.get_position().x - _b.get_position().x)) +
           std::abs((_a.get_position().y - _b.get_position().y));
  }

  //----------------------------------------------------------------------------
  // GET NODE FROM LIST
  //----------------------------------------------------------------------------
  path_node get_node_from_closed_list(std::vector<path_node> closed_list,
                                      cv::Point _coordinate,
                                      int _debug = NO_DEBUG) {
    const float flt_max = 9999.9f;
    for (size_t i = 0; i < closed_list.size(); i++) {
      if (closed_list[i].get_position() == _coordinate) {
        if (_debug) {
          std::cout << "returns node at: " << closed_list[i].get_position()
                    << std::endl;
        }
        return closed_list[i];
      }
    }
    if (_debug) {
      std::cout << "NO NODE FOUND -> RETURNS INVALID NODE" << std::endl;
    }
    /*
     * Invalid nodes are intentionally handeled by the backtracker, because not
     * all nodes are in the closed_list. Their g_cose are also set to a high
     * value to make sure they dont interfere with the backtracker.
     * */
    path_node invalid_node = path_node({-1, -1});
    invalid_node.g_cost = flt_max;
    return invalid_node;
  }

public:
  int map_width;
  int map_height;
  //----------------------------------------------------------------------------
  // CONSTRUCTOR
  //----------------------------------------------------------------------------
  path_planning(cv::Mat _input_map, std::string _name = "")
      : input_map(_input_map), node_map(_input_map), eroded_map(_input_map),
        map_name(_name), map_width(_input_map.cols),
        map_height(_input_map.rows){};

  //----------------------------------------------------------------------------
  // SETUP RANDOM GENERATOR
  //----------------------------------------------------------------------------
  void random_generator_setup() { srand(time(NULL)); }

  //----------------------------------------------------------------------------
  // SHOW THE NODE MAP
  //----------------------------------------------------------------------------
  void show_map(float _scale_size = 1, bool _wait = NO_WAIT) {
    /*guard clause*/
    if (node_map.empty()) {
      return;
    }
    /*clone map insead of using implicit pointers*/
    cv::Mat view_map = node_map.clone();
    cv::resize(view_map, view_map, cv::Size(), _scale_size, _scale_size,
               cv::INTER_NEAREST);
    cv::namedWindow(map_name);
    cv::imshow(map_name, view_map);
    if (_wait) {
      cv::waitKey();
    }
  }

  //----------------------------------------------------------------------------
  // RESIZE THE IN MEMORY MAP
  //----------------------------------------------------------------------------
  void resize_map(int _scale_size) {
    cv::resize(node_map, node_map, cv::Size(), _scale_size, _scale_size,
               cv::INTER_NEAREST);
    map_width = node_map.cols;
    map_height = node_map.rows;
    scaling_factor = _scale_size;
  }

  //----------------------------------------------------------------------------
  // GENERATE QUASIRANDOM HAMMERSLEY NODES AND SAVE
  //----------------------------------------------------------------------------
  void generate_quasirandom_hammersley_nodes(int _node_number) {
    const int max_map_width = 1;
    const float half_map_width = 0.5;
    float displacement;
    float width_position = 0, height_position = 0;
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
  // ERODE MAP
  //----------------------------------------------------------------------------
  void erode_map(int _erosion_count = 0) {
    eroded_map = node_map.clone();
    cv::Mat structuring_element(5, 5, CV_8U, cv::Scalar(1));
    for (int i = 0; i < _erosion_count; i++) {
      cv::erode(eroded_map, eroded_map, structuring_element);
    }
  }

  //----------------------------------------------------------------------------
  // PAD WALLS WITH A BRUSHFIRE LIKE ALGORITHM
  //----------------------------------------------------------------------------
  void pad_walls(int _padding_pixels = 1, int _debug = NO_DEBUG) {
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

    return;

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
  // ADD ROOM NODES
  //----------------------------------------------------------------------------
  void add_room_nodes(std::vector<cv::Point> _room_node_coordinates) {
    for (size_t i = 0; i < _room_node_coordinates.size(); i++) {
      cv::Point upscaled_coordinates = {
          _room_node_coordinates[i].x * scaling_factor,
          _room_node_coordinates[i].y * scaling_factor};

      waypoint_nodes.push_back(upscaled_coordinates);
    }
    for (size_t i = 0; i < _room_node_coordinates.size(); i++) {

      cv::Point upscaled_coordinates = {
          _room_node_coordinates[i].x * scaling_factor,
          _room_node_coordinates[i].y * scaling_factor};
      path_node current_node = path_node(upscaled_coordinates);
      for (size_t j = 0; j < waypoint_nodes.size(); j++) {
        if (valid_bird_path(current_node.get_position(), waypoint_nodes[j])) {
          current_node.add_possible_path(waypoint_nodes[j]);
        }
      }
      room_nodes.push_back(current_node);
      waypoint_path_nodes.push_back(current_node);
    }
  }

  //----------------------------------------------------------------------------
  // REMOVE UNWANTED AND INVALID NODES FROM THE NODE LIST
  //----------------------------------------------------------------------------
  void remove_unwanted_nodes(bool _debug = NO_DEBUG) {
    std::vector<cv::Point> valid_waypoint_nodes = {};
    for (size_t i = 0; i < waypoint_nodes.size(); i++) {

      if (get_eroded_pixel_color(waypoint_nodes[i]) != black_pixel) {
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
    for (size_t i = 0; i < room_nodes.size(); i++) {
      cv::circle(node_map, room_nodes[i].get_position(), 10, GREEN, cv::FILLED);
    }
  }

  //----------------------------------------------------------------------------
  // FIND IF A BIRD PATH IS POSSIBLE
  //----------------------------------------------------------------------------
  int valid_bird_path(cv::Point _start, cv::Point _goal,
                      bool _debug = NO_DEBUG) {

    if (_start == _goal) {
      return -1; /*Disaster*/
    }
    cv::LineIterator bird_path(eroded_map, _start, _goal, 8);

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

          i_current_node.add_possible_path(j_current_node.get_position());
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
                 waypoint_path_nodes[i].get_possible_paths()[j], _color);
      }
    }
  }
  //----------------------------------------------------------------------------
  // A* PATHFINDING
  //----------------------------------------------------------------------------
  std::vector<cv::Point> a_star_path_finder(cv::Point _start, cv::Point _goal,
                                            int _debug = NO_DEBUG) {
    /*Initialization*/
    const float flt_max = 9999.9f;
    std::vector<cv::Point> result = {};
    std::vector<path_node> open_list;
    std::vector<path_node> closed_list;
    _start = {_start.x * scaling_factor, _start.y * scaling_factor};
    _goal = {_goal.x * scaling_factor, _goal.y * scaling_factor};
    if (_debug) {
      std::cout << "upscaling factor: " << scaling_factor
                << " start: " << _start << " end: " << _goal << std::endl;
    }

    open_list.push_back(get_node_from_coordinate(_start));

    /*Test all possible paths*/
    while (!open_list.empty()) {

      /*Find lowest f_cost node in open_list*/
      int index = 0;
      float lowest_f = flt_max;
      for (size_t i = 0; i < open_list.size(); i++) {
        if (open_list[i].f_cost <= lowest_f) {
          index = i;
          lowest_f = open_list[i].f_cost;
          if (_debug) {
            std::cout << "while loop iteration" << std::endl;
          }
        }
      }

      /*Make current node the lowest f_cost node in open_list*/
      path_node current_node = open_list[index];
      closed_list.push_back(open_list[index]);
      open_list.erase(open_list.begin() + index);

      if (_debug) {
        std::cout << "current node found" << std::endl;
      }

      /*Exit condition*/
      if (current_node.get_position() == _goal) {

        /*Backtrack to get path*/
        path_node current_valid_node = current_node;

        /*find the path that has the lowest g_cost*/
        while (current_valid_node.get_position() != _start) {

          /*find the child with the smallest g_cost and make it the
           * current_valid_node*/
          float lowest_g = current_node.g_cost;
          int valid_index = -1;
          for (size_t i = 0; i < current_valid_node.get_possible_paths().size();
               i++) {
            if (get_node_from_closed_list(
                    closed_list, current_valid_node.get_possible_paths()[i])
                    .g_cost < lowest_g) {
              if (_debug) {
                std::cout << "current g_cost is lower than lowest_g"
                          << std::endl
                          << "current g_cost = " << lowest_g << std::endl;
              }

              lowest_g =
                  get_node_from_closed_list(
                      closed_list, current_valid_node.get_possible_paths()[i])
                      .g_cost;
              valid_index = i;
            }
          }

          /*add the child with the smallest g_cost to the resulting path*/
          result.push_back(current_valid_node.get_position());
          if (_debug) {
            std::cout << "added node to backtracking path: "
                      << current_valid_node.get_position()
                      << " g_cost: " << current_valid_node.g_cost << std::endl;
          }
          current_valid_node = get_node_from_closed_list(
              closed_list,
              current_valid_node.get_possible_paths()[valid_index]);
        }

        /*add the starting position*/
        result.push_back(current_valid_node.get_position());
        if (_debug) {
          std::cout << "A* PATH FOUND" << std::endl;
          for (size_t i = 0; i < result.size(); i++) {
            std::cout << result[i] << std::endl;
          }
        }
        return result;
      }

      /*Generate children for the current_node*/
      for (size_t i = 0; i < current_node.get_possible_paths().size(); i++) {

        if (_debug) {
          std::cout << "child iteration " << i << std::endl;
        }

        /*does the child already exist?
         * -> has the child generated new children?*/
        bool is_in_closed_list = false;
        for (size_t j = 0; j < closed_list.size(); j++) {
          if (current_node.get_possible_paths()[i] ==
              closed_list[j].get_position()) {
            is_in_closed_list = true;
          }
        }

        if (_debug && is_in_closed_list) {
          std::cout << "child exists in closed_list" << std::endl;
        }

        /*if the child does not exist in closed_list*/
        if (!is_in_closed_list) {

          /*generate new child*/
          path_node child =
              get_node_from_coordinate(current_node.get_possible_paths()[i]);
          child.g_cost = current_node.g_cost + heuristic(child, current_node);
          child.h_cost = heuristic(child, get_node_from_coordinate(_goal));
          child.f_cost = child.g_cost + child.h_cost;

          /*is the new child already added to open_list*/
          bool is_in_open_list = false;
          for (size_t j = 0; j < open_list.size(); j++) {
            if (child.get_position() == open_list[j].get_position()) {
              is_in_open_list = true;

              /*if the new child cost is better than the already existing
               * child -> replace it*/
              if (child.g_cost < open_list[j].g_cost) {
                open_list.push_back(child);
                open_list.erase(open_list.begin() + j);
                if (_debug) {
                  std::cout << "child exists in open_list and child does not "
                            << std::endl;
                }
              } else {
                if (_debug) {
                  std::cout << "child exists in open_list and child has higher "
                               "g_cost than the already existing child"
                            << std::endl;
                }
              }
            }
          }

          /*new child isint inside open_list*/
          if (!is_in_open_list) {

            if (_debug) {
              std::cout << "child does not exist in open_list -> push it onto "
                           "open_list"
                        << std::endl;
            }
            /*add new child*/
            open_list.push_back(child);
          } else {
          }
        }
      }
    }
    if (_debug) {
      std::cout << "NO PATH FOUND" << std::endl;
    }
    return result;
  }

  //----------------------------------------------------------------------------
  // A* PATH DRAWER
  //----------------------------------------------------------------------------
  void draw_a_star_path(std::vector<cv::Point> a_star_path,
                        cv::Vec3b _color = red_pixel) {
    for (size_t i = 1; i < a_star_path.size(); i++) {
      cv::line(node_map, a_star_path[i], a_star_path[i - 1], _color, 4);
    }
    return;
  }
  //----------------------------------------------------------------------------
  // STATIC ALL IN ONE FUNCTION THAT FINDS A PATH ON A IMAGE AND RETURNS A SET
  // OF COORDINATES CALCULATED BY AN A* ALGORITHM
  //----------------------------------------------------------------------------
  static std::vector<cv::Point>
  calculate_a_star_path(cv::Mat _image, std::vector<cv::Point> room_nodes,
                        int hammersley_node_amount, int map_resize_scalar,
                        int map_erosion, float map_show_scalar) {
    path_planning path(_image);

    path.resize_map(map_resize_scalar);
    path.show_map(map_show_scalar, NO_WAIT);
    path.erode_map(map_erosion);
    path.generate_quasirandom_hammersley_nodes(hammersley_node_amount);
    path.add_room_nodes(room_nodes);
    path.remove_unwanted_nodes();
    path.color_waypoint_nodes(blue_pixel);
    path.find_node_map_connections();
    path.draw_node_map_connections(red_pixel);
    path.color_waypoint_nodes(blue_pixel);
    std::vector<cv::Point> a_star_path =
        path.a_star_path_finder(room_nodes[3], room_nodes[16]);

    path.draw_a_star_path(a_star_path, blue_pixel);
    path.show_map(map_show_scalar, WAIT);
    return a_star_path;
  }
};
