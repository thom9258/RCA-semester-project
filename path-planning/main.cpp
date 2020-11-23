#include "path_planning.h"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <stdexcept>
#include <stdio.h>

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  const float map_show_scalar = 0.7;

  std::vector<cv::Mat> maps = {};
  maps.push_back(cv::imread("./maps/floor_plan_small.png", 0));
  maps.push_back(cv::imread("./maps/floor_plan_large.png", 0));

  path_planning path(maps[1]);

  path.resize_map(10);
  path.show_map(map_show_scalar, WAIT);

  path.pad_walls(200, DEBUG);
  /*
    path.generate_quasirandom_hammersley_nodes(80);
    path.remove_unwanted_nodes();
    path.color_waypoint_nodes(blue_pixel);
    path.show_map(map_show_scalar, WAIT);
    path.find_node_map_connections();
    path.draw_node_map_connections(green_pixel);
    path.color_waypoint_nodes(blue_pixel);
  */
  std::cout << "PADDED" << std::endl;
  path.show_map(map_show_scalar, WAIT);

  return 0;
}
