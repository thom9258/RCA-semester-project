#include "path_planning.h"
#include <iostream>
//#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>
#include <stdio.h>

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  cv::Mat img = cv::imread("./maps/floor_plan_small.png");
  path_planning path(img, "name");

  path.resize_map(6);

  path.show_map(4, WAIT);

  path.generate_quasirandom_hammersley_nodes(200);
  path.remove_unwanted_nodes(NODEBUG);
  path.color_waypoint_nodes(blue_pixel);

  path.show_map(4, WAIT);

  path.print_waypoint_nodes();
  path.find_node_map_connections();
  //  path.print_node_map_connections();
  path.draw_node_map_connections(green_pixel);
  path.color_waypoint_nodes(blue_pixel);

  path.show_map(4, WAIT);

  return 0;
}
