#include "localization.h"
#include "path_planning.h"
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>
#include <stdio.h>

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  bool do_path_planning = false;

  const float map_show_scalar = 0.7;
  int hammersley_node_amount = 1000;
  int map_resize_scalar = 10;
  int map_erosion = 3;
  std::vector<cv::Mat> maps = {};
  maps.push_back(cv::imread("./maps/floor_plan_small.png"));
  maps.push_back(cv::imread("./maps/floor_plan_large.png"));
  std::vector<cv::Point> room_nodes = {
      {60, 40}, {35, 65}, {10, 65},  {10, 10}, {25, 10},  {10, 25},
      {45, 10}, {65, 10}, {65, 25},  {95, 20}, {110, 20}, {110, 50},
      {90, 50}, {60, 55}, {110, 70}, {60, 70}, {80, 70}};

  if (do_path_planning) {
    path_planning path(maps[1]);
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
  } else {
    /**************************************************************************/
    cv::Point start_position = {0, 0};
    localization dr = localization(start_position);
    dr.max_error = 0.0f;
    for (size_t i = 0; i < 10; i++) {
      matrix::print(dr.kalman->kalman_gain());
      std::cout << "determinant: "
                << matrix::determinant(dr.kalman->kalman_gain()) << std::endl;
      dr.update_dead_reckoning(1, 0.2, DEBUG);
    }
  }
  return 0;
}
