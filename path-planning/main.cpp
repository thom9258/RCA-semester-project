#include "localization.h"
#include "path_planning.h"
#include <fstream>
#include <iostream>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>
#include <stdio.h>
void hammersley_node_test(void) { return; }
int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  bool do_hammersley_node_test = false;
  bool do_node_validation_test = false;
  bool a_star_test_one_to_all = false;
  bool a_star_test_full_path = false;
  bool a_star_test_back_and_forth = false;
  bool qq_plot_normal_distibution = false;
  bool do_room_node = true;

  bool do_path_planning = false;
  bool do_localization = false;

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

  /*****************************************************************************
   * ROOM NODE TEST
   * **************************************************************************/
  if (do_room_node) {
    path_planning p(maps[1]);
    p.resize_map(map_resize_scalar);
    p.show_map(map_show_scalar, NO_WAIT);
    p.erode_map(map_erosion);
    p.add_room_nodes(room_nodes);
    p.color_waypoint_nodes();

    p.show_map(map_show_scalar, WAIT);
    p.save_map("room_nodes.png");
  }
  /*****************************************************************************
   * HAMMERSLEY NODE TEST
   * **************************************************************************/
  if (do_hammersley_node_test) {

    path_planning p(maps[1]);
    p.resize_map(map_resize_scalar);
    p.show_map(map_show_scalar, NO_WAIT);
    p.erode_map(map_erosion);

    p.generate_quasirandom_hammersley_nodes(1000);
    p.remove_unwanted_nodes();
    p.color_waypoint_nodes(red_pixel, 4);
    p.waypoint_nodes = {};
    p.save_map("hammersley_node_placement_1000_red.png");
    p.show_map(map_show_scalar, WAIT);

    p.generate_quasirandom_hammersley_nodes(100);
    p.remove_unwanted_nodes();
    p.color_waypoint_nodes(blue_pixel, 8);
    p.waypoint_nodes = {};
    p.save_map("hammersley_node_placement_100_green.png");
    p.show_map(map_show_scalar, WAIT);

    p.generate_quasirandom_hammersley_nodes(10);
    p.remove_unwanted_nodes();
    p.color_waypoint_nodes(blue_pixel, 15);
    p.waypoint_nodes = {};
    //    p.save_map("hammersley_node_placement_10_blue.png");
    p.show_map(map_show_scalar, WAIT);
  }

  /*****************************************************************************
   * NODE VALIDATION TEST
   * **************************************************************************/
  if (do_node_validation_test) {
    path_planning p(maps[1]);
    p.resize_map(map_resize_scalar);
    p.show_map(map_show_scalar, NO_WAIT);
    p.erode_map(map_erosion);
    p.show_erosion_map(map_show_scalar, WAIT);
    p.save_map("obstacle_padding.png", "ERODED");

    p.generate_quasirandom_hammersley_nodes(1000);
    p.remove_unwanted_nodes();
    p.color_waypoint_nodes(red_pixel, 4);
    //    p.show_map(map_show_scalar, WAIT);
    p.find_node_map_connections();
    p.draw_node_map_connections(grey_pixel);
    p.color_waypoint_nodes(red_pixel, 4);
    p.show_map(map_show_scalar, WAIT);
    p.save_map("node_validation_and_test_pathing.png");
  }

  /*****************************************************************************
   * NODE VALIDATION TEST
   * **************************************************************************/
  if (a_star_test_one_to_all) {
    std::vector<cv::Point> room_nodes = {
        {60, 40}, {35, 65}, {10, 65},  {10, 10}, {25, 10},  {10, 25},
        {45, 10}, {65, 10}, {65, 25},  {95, 20}, {110, 20}, {110, 50},
        {90, 50}, {60, 55}, {110, 70}, {80, 70}, {60, 70}};

    path_planning p(maps[1]);
    p.resize_map(map_resize_scalar);
    p.show_map(map_show_scalar, NO_WAIT);
    p.erode_map(map_erosion);

    p.generate_quasirandom_hammersley_nodes(1000);
    p.add_room_nodes(room_nodes);
    p.remove_unwanted_nodes();
    p.color_waypoint_nodes(red_pixel, 4);
    //    p.show_map(map_show_scalar, WAIT);
    p.find_node_map_connections();
    p.draw_node_map_connections(grey_pixel);
    p.color_waypoint_nodes(red_pixel, 4);
    //    cv::Vec3b curr_color = blue_pixel;
    //    float modifier = 1;

    for (size_t i = 0; i < room_nodes.size(); i++) {
      std::vector<cv::Point> a_star_path =
          p.a_star_path_finder(room_nodes[0], room_nodes[i]);
      p.draw_a_star_path(a_star_path, blue_pixel);
      //      p.draw_a_star_path(a_star_path, curr_color);
      //      cv::Vec3b curr_color = {uchar(float(blue_pixel[0] * modifier)),
      //                              uchar(float(blue_pixel[1] * modifier)),
      //                              uchar(float(blue_pixel[2] * modifier))};
      //      modifier = modifier * 0.8;
    }
    p.save_map("a_star_test_one_to_all.png");
    p.show_map(map_show_scalar, WAIT);
  }
  if (a_star_test_full_path) {
    std::vector<cv::Point> room_nodes = {
        {60, 40}, {35, 65}, {10, 65},  {10, 10}, {25, 10},  {10, 25},
        {45, 10}, {65, 10}, {65, 25},  {95, 20}, {110, 20}, {110, 50},
        {90, 50}, {60, 55}, {110, 70}, {80, 70}, {60, 70}};

    path_planning p(maps[1]);
    p.resize_map(map_resize_scalar);
    p.show_map(map_show_scalar, NO_WAIT);
    p.erode_map(map_erosion);

    p.generate_quasirandom_hammersley_nodes(1000);
    p.add_room_nodes(room_nodes);
    p.remove_unwanted_nodes();
    p.color_waypoint_nodes(red_pixel, 4);
    p.find_node_map_connections();
    p.draw_node_map_connections(grey_pixel);
    p.color_waypoint_nodes(red_pixel, 4);

    for (size_t i = 1; i < room_nodes.size(); i++) {
      std::vector<cv::Point> a_star_path =
          p.a_star_path_finder(room_nodes[i - 1], room_nodes[i]);
      p.draw_a_star_path(a_star_path, blue_pixel);
    }

    p.save_map("a_star_test_full_path.png");
    p.show_map(map_show_scalar, WAIT);
  }
  if (a_star_test_back_and_forth) {
    std::vector<cv::Point> room_nodes = {
        {60, 40}, {35, 65}, {10, 65},  {10, 10}, {25, 10},  {10, 25},
        {45, 10}, {65, 10}, {65, 25},  {95, 20}, {110, 20}, {110, 50},
        {90, 50}, {60, 55}, {110, 70}, {80, 70}, {60, 70}};

    path_planning p(maps[1]);
    p.resize_map(map_resize_scalar);
    p.show_map(map_show_scalar, NO_WAIT);
    p.erode_map(map_erosion);

    p.generate_quasirandom_hammersley_nodes(1000);
    p.add_room_nodes(room_nodes);
    p.remove_unwanted_nodes();
    p.color_waypoint_nodes(red_pixel, 4);
    p.find_node_map_connections();
    p.draw_node_map_connections(grey_pixel);
    p.color_waypoint_nodes(red_pixel, 4);

    std::vector<cv::Point> a_star_path =
        p.a_star_path_finder(room_nodes[3], room_nodes[16]);
    p.draw_a_star_path(a_star_path, blue_pixel, 12);

    std::vector<cv::Point> a_star_path_2 =
        p.a_star_path_finder(room_nodes[3], room_nodes[16]);
    p.draw_a_star_path(a_star_path_2, green_pixel);
    p.save_map("a_star_test_back_and_forth.png");
    p.show_map(map_show_scalar, WAIT);
  }
  /*****************************************************************************
   * QUASI_RANDOM QQ-PLOT
   * **************************************************************************/
  if (qq_plot_normal_distibution) {
    path_planning p(maps[1]);
    p.resize_map(map_resize_scalar);
    p.show_map(map_show_scalar, NO_WAIT);

    p.generate_quasirandom_hammersley_nodes(1000);
    std::vector<cv::Point> data_set = p.waypoint_nodes;
    std::ofstream outputfile;
    outputfile.open("quasi_random_normal_distibution_test.csv");
    for (size_t i = 0; i < data_set.size(); i++) {
      outputfile << data_set[i].x << "," << data_set[i].y << std::endl;
    }
  }

  /*****************************************************************************
   * PATH PLANNING
   * **************************************************************************/
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
  }
  /*****************************************************************************
   * LOCALIZATION
   * **************************************************************************/
  if (do_localization) {
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
    for (size_t i = 0; i < 10; i++) {
      matrix::print(dr.kalman->kalman_gain());
      std::cout << "determinant: "
                << matrix::determinant(dr.kalman->kalman_gain()) << std::endl;
      dr.update_dead_reckoning(0, 0, DEBUG);
    }
  }
  return 0;
}
