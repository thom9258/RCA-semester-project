#pragma once
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

#define DEBUG 1
#define NODEBUG 0

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

public:
  path_planning(cv::Mat _input_map, std::string _name = "")
      : input_map(_input_map), map_name(_name), map_width(_input_map.cols),
        map_height(_input_map.rows){};

  void show_input_map(float _scale_size = 1) {
    if (input_map.empty()) {
      std::cerr << "could not load map" << std::endl;
      return;
    }
    cv::resize(input_map, input_map, cv::Size(), _scale_size, _scale_size,
               cv::INTER_NEAREST);
    cv::namedWindow(map_name);
    cv::imshow(map_name, input_map);
  }

  void random_generator_setup() { srand(time(NULL)); }

  void generate_random_nodes() {
    // https://www.rajgunesh.com/resources/downloads/statistics/samplingmethods.pdf
    // Page 203 (chapter 7.1.1) in the book from Robots in Context
    // Page 220 --||--
    // 1. Random waypoint assignment algorithm
    // 2. Invalid waypoint elimination
    // 3. Density waypoint elimination
    // 4. waypoint path connection
    // 5. path density elimination
    // return waypoint_node vector and image with drawn on waypoints and paths.
  }
};
