#include "path_planning.h"
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>
#include <stdio.h>

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  const float map_show_scalar = 0.7;

  std::vector<cv::Mat> maps = {};
  maps.push_back(cv::imread("./maps/floor_plan_small.png"));
  maps.push_back(cv::imread("./maps/floor_plan_large.png"));

  std::vector<cv::Point> room_nodes = {
      {60, 40}, {35, 65}, {10, 65},  {10, 10}, {25, 10},  {10, 25},
      {45, 10}, {65, 10}, {65, 25},  {95, 20}, {110, 20}, {110, 50},
      {90, 50}, {60, 55}, {110, 70}, {60, 70}, {80, 70}};

  path_planning path(maps[1]);
  std::cout << "width: " << path.map_width << " height: " << path.map_height
            << std::endl;

  path.resize_map(10);
  path.show_map(map_show_scalar, WAIT);

  path.erode_map(3);
  //  path.show_map(map_show_scalar, WAIT);
  path.generate_quasirandom_hammersley_nodes(80);
  path.add_room_nodes(room_nodes);
  std::cout << "ADD ROOM NODES" << std::endl;
  path.remove_unwanted_nodes();
  std::cout << "REMOVE UNWANTED WAYPOINT NODES" << std::endl;
  path.color_waypoint_nodes(blue_pixel);

  path.show_map(map_show_scalar, WAIT);

  std::cout << "COLOR WAYPOINT NODES" << std::endl;
  path.find_node_map_connections();
  path.draw_node_map_connections(red_pixel);
  path.color_waypoint_nodes(blue_pixel);
  std::vector<cv::Point> a_star_path =
      path.a_star_path_finder(room_nodes[0], room_nodes[10], DEBUG);
  path.show_map(map_show_scalar, WAIT);

  return 0;
}
