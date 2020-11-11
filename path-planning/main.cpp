#include "path_planning.h"
#include <iostream>
//#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>
#include <stdio.h>

//  Description:    (jump-tag)!
//
//  Developer:      Thomas Alexgaard Jensen
//  Creation date:  (jump-tag)!
//
//  Changelog:      DDMMYY  Change
//                  (jump-tag)!

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  cv::Mat img = cv::imread("./maps/floor_plan_small.png");
  path_planning path(img, "name");

  //  path.resize_map(6);
  path.generate_quasirandom_hammersley_nodes(10);
  path.print_waypoint_nodes();
  path.remove_unwanted_nodes(DEBUG);
  path.color_waypoint_nodes();
  path.print_waypoint_nodes();
  path.show_map(8);

  cv::waitKey();
  return 0;
}
