#pragma once
#include "path_planning.h"
#include <bits/c++config.h>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <math.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <stdexcept>
#include <stdio.h>
/*
 * Description:     A localization class implementation for the project
 *                  RCA 5.semester.
 *
 * Creation date:   261120
 *
 * Changelog:       DDMMYY  XX        Change
 *                  261120  TH/MI/ER  Implemented Dead reckoning
 */
#define PI 3.14f

/*******************************************************************************
 *******************************************************************************
 * LOCALIZATION CLASS
 *******************************************************************************
 ******************************************************************************/
class localization {
private:
  cv::Point2f x_vector = {0, 0};
  float previous_velocity = 0;
  float previous_distance = 0;
  float previous_direction = 0;
  float distance_traveled = 0;
  const float MAX_ROTATION = PI;
  const float MIN_ROTATION = -PI;
  const float TOTAL_ROTATION = 2 * PI;

public:
  localization(cv::Point _x_start_position = {0, 0})
      : x_vector(_x_start_position){};

  /*****************************************************************************
   * CALCULATE DEAD RECKONING
   ****************************************************************************/
  void update_dead_reckoning(float _velocity, float _direction,
                             int _debug = NO_DEBUG) {
    previous_distance = distance_traveled;
    distance_traveled = previous_velocity + _velocity;

    float rotation = previous_direction + _direction;
    while (rotation > MAX_ROTATION) {
      rotation -= TOTAL_ROTATION;
    }
    while (rotation < MIN_ROTATION) {
      rotation += TOTAL_ROTATION;
    }
    previous_velocity = distance_traveled;
    previous_direction = rotation;

    cv::Point2f rotation_vector = {
        cos(rotation) * (distance_traveled - previous_distance),
        sin(rotation) * (distance_traveled - previous_distance)};

    x_vector = {x_vector.x + rotation_vector.x, x_vector.y + rotation_vector.y};
    if (_debug) {
      std::cout << "velocity: " << _velocity << " direction: " << _direction
                << std::endl
                << x_vector << " " << rotation;
    }
  }
  /*****************************************************************************
   * RETURN X VECTOR FOR DEBUGGING
   ****************************************************************************/
  cv::Point2f get_x_vector(void) { return x_vector; }
};
