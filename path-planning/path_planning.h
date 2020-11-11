#pragma once
#include <cstddef>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <stdio.h>

//  Description:    A path planning class implementation for the project
//                  RCA 5.semester.
//
//  Developer:
//  Creation date:  281020
//
//  Changelog:      DDMMYY  XX  Change
//                  281020  TH  Started construction of the path planning class
//

//  NOTES:
//  A Hammersley or Halton sequence might be good for a sampling strategy.
//  Algorithms such as RRT (Rapidly-exploring Random Tree) with a merge might be
//  good but also complicated (found on page ~240)
//  https://www.researchgate.net/publication/244441430_Sampling_with_Hammersley_and_Halton_Points

#define DEBUG 1
#define NODEBUG 0

#define GREYSCALE 0
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

class path_planning {
private:
  cv::Mat input_map;
  cv::Mat node_map;
  std::string map_name;
  std::vector<cv::Point> waypoint_nodes;
  int map_width;
  int map_height;

  cv::Point generate_random_point(bool _debug = NODEBUG) {
    cv::Point result = {(rand() % map_width), (rand() % map_height + 1)};
    if (_debug) {
      std::cout << "Generated point: " << result << std::endl;
    }
    return result;
  }

  cv::Vec3b get_pixel_bgr(cv::Point _point)
  //  returns in the format cv::Vec3b [a, b, c]
  //  where white = [255, 255, 255] and black = [0, 0, 0] ect.
  {
    return node_map.at<cv::Vec3b>(_point.x, _point.y);
  }

public:
  path_planning(cv::Mat _input_map, std::string _name = "")
      : input_map(_input_map), node_map(_input_map), map_name(_name),
        map_width(_input_map.cols), map_height(_input_map.rows){};

  void show_map(float _scale_size = 1) {
    if (node_map.empty()) {
      std::cerr << "could not load map" << std::endl;
      return;
    }
    cv::resize(node_map, node_map, cv::Size(), _scale_size, _scale_size,
               cv::INTER_NEAREST);
    cv::namedWindow(map_name);
    cv::imshow(map_name, node_map);
  }

  void resize_map(int _scale_size) {
    cv::resize(node_map, node_map, cv::Size(), _scale_size, _scale_size,
               cv::INTER_NEAREST);
    map_width = node_map.cols;
    map_height = node_map.rows;
  }

  //  void random_generator_setup() { srand(time(NULL)); }

  // https://www.rajgunesh.com/resources/downloads/statistics/samplingmethods.pdf
  // Page 203 (chapter 7.1.1) in the book from Robots in Context
  // Page 220 --||--
  // 1. Random waypoint assignment algorithm
  // 2. Invalid waypoint elimination
  // 3. Density waypoint elimination
  // 4. waypoint path connection
  // 5. path density elimination
  // return waypoint_node vector and image with drawn on waypoints and paths.
  void generate_quasirandom_hammersley_nodes(int _node_number) {
    const int max_map_width = 1;
    const float half_map_width = 0.5;
    float displacement;
    float width_position, height_position;
    int current_node, current_node_expansion;

    // generate n nodes for the map
    for (current_node = 0; current_node < _node_number; current_node++) {
      width_position = 0;

      // calculate the displacement of the node
      for (displacement = max_map_width, current_node_expansion = current_node;
           displacement *= half_map_width; current_node_expansion >>= 1) {

        // check if current_node_expansion is odd
        if ((current_node_expansion % 2) == 1) {

          // calculate the width and height of the current node
          width_position += displacement;
          height_position = (current_node + half_map_width) / _node_number;

          // upscale calculated waypoint to fit node_map and push into
          // waypoint_nodes vector
          waypoint_nodes.push_back({int(width_position * map_width),
                                    int(height_position * map_height)});
        }
      }
    }
  }

  void remove_unwanted_nodes(bool _debug = NODEBUG) {
    std::vector<cv::Point> valid_waypoint_nodes = {};
    for (size_t i = 0; i < waypoint_nodes.size(); i++) {

      if (get_pixel_bgr(waypoint_nodes[i]) != black_pixel) {
        //        waypoint_nodes.erase(waypoint_nodes.begin() + i);
        valid_waypoint_nodes.push_back(waypoint_nodes[i]);
      } else {
        if (_debug) {
          std::cout << "removed node: " << i << " " << waypoint_nodes[i]
                    << std::endl;
          cv::line(node_map, waypoint_nodes[i], waypoint_nodes[i], red_pixel);
        }
      }
    }
    waypoint_nodes = valid_waypoint_nodes;
  }

  void print_waypoint_nodes(void) {
    std::cout << waypoint_nodes.size() << " nodes:" << std::endl;
    for (size_t i = 0; i < waypoint_nodes.size(); i++) {
      std::cout << waypoint_nodes[i] << " ";
    }
    std::cout << std::endl;
  }

  void color_waypoint_nodes(cv::Vec3b _color = green_pixel) {
    for (size_t i = 0; i < waypoint_nodes.size(); i++) {
      cv::line(node_map, waypoint_nodes[i], waypoint_nodes[i], _color);
    }
  }
};
