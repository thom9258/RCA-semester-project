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

  int image_scaler = 20;
  path.show_input_map(image_scaler);
  path.random_generator_setup();
  path.generate_random_nodes();
  cv::waitKey();
  return 0;
}
