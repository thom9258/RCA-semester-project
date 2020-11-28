#pragma once
#include "matrix.h"
#include "path_planning.h"
#include <bits/c++config.h>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <random>
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
 *                  271120  TH/MI/ER  Started implementation of kalman filtering
 *                  281120  TH/MI/ER  Continued work on kalman filtering, adding
 *                                    randomness for tests
 *
 */
#define PI 3.14f

/*******************************************************************************
 *******************************************************************************
 * KALMAN FILTER CLASS
 *******************************************************************************
 ******************************************************************************/
class kalman_filter {
private:
  //  const double h_measurement_map_scalar = 1.0f;
  unsigned long long timestep = 0;
  const float q_value = 40.0f;
  const float r_value = 10.0f;
  std::vector<std::vector<double>> q_estimated_covariance = {
      {q_value, 0, 0}, {0, q_value, 0}, {0, 0, q_value}};
  std::vector<std::vector<double>> A = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  std::vector<std::vector<double>> B = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  std::vector<std::vector<double>> C = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  std::vector<std::vector<double>> D = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

public:
  std::vector<std::vector<double>> p_error_covariance = {
      {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  std::vector<std::vector<double>> k_kalman_gain = {
      {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  std::vector<std::vector<double>> x_hat_estimated_state = {};     /*x_k*/
  std::vector<std::vector<double>> new_x_hat_estimated_state = {}; /*x_{k+1}*/
  std::vector<std::vector<double>> r_noise_covariance = {
      {r_value, 0, 0}, {0, r_value, 0}, {0, 0, r_value}};

  /*****************************************************************************
   * CONSTRUCTOR
   ****************************************************************************/
  kalman_filter(){};
  kalman_filter(std::vector<std::vector<double>> _q,
                std::vector<std::vector<double>> _p,
                std::vector<std::vector<double>> _x,
                std::vector<std::vector<double>> _k,
                std::vector<std::vector<double>> _r)
      : q_estimated_covariance(_q), p_error_covariance(_p), k_kalman_gain(_k),
        x_hat_estimated_state(_x), new_x_hat_estimated_state(_x),
        r_noise_covariance(_r){};

  /*****************************************************************************
   * A-KC MATRIX FOR DEBUGGING
   ****************************************************************************/
  std::vector<std::vector<double>> kalman_gain(void) {
    return matrix::subtract(A, matrix::multiply(k_kalman_gain, C));
  }
  /*****************************************************************************
   * UPDATE KALMAN FILTER
   ****************************************************************************/
  void calculate_kalman(std::vector<std::vector<double>> _u_input,
                        std::vector<std::vector<double>> _y_result,
                        int _debug = NO_DEBUG) {

    if (_debug) {
      std::cout << "calculate kalman start" << std::endl;
    }

    /*************************Prediction step**********************************/

    /*
     * x_hat_{k+1} = A * x_hat_k + B*u
     * */
    new_x_hat_estimated_state =
        matrix::add((matrix::multiply(A, x_hat_estimated_state)),
                    (matrix::multiply(B, _u_input)));

    if (_debug) {
      std::cout << "estimated new x hat" << std::endl;
    }
    /*
     * P_{k+1|k} = A*P_{k|k}*transpose(A) + Q
     * */
    std::vector<std::vector<double>> AP =
        matrix::multiply(A, p_error_covariance);
    std::vector<std::vector<double>> AT = matrix::transpose(A);
    p_error_covariance =
        matrix::add((matrix::multiply(AP, AT)), q_estimated_covariance);

    if (_debug) {
      std::cout << "calculated P_{k+1|k}" << std::endl;
    }
    /*************************Update step**************************************/

    /*
     * x_{k+1} = x_{k+1} + K * (y - C * x_{k+1})
     * */
    /* (y - C * x_{k+1}) */
    std::vector<std::vector<double>> Y_CX = matrix::subtract(
        _y_result, matrix::multiply(C, new_x_hat_estimated_state));
    /* K * (y - C * x_{k+1}) */
    std::vector<std::vector<double>> KY_CX =
        matrix::multiply(k_kalman_gain, Y_CX);
    /*x_{k+1} = x_{k+1} + K * (y - C * x_{k+1})*/
    new_x_hat_estimated_state = matrix::add(new_x_hat_estimated_state, KY_CX);

    if (_debug) {
      std::cout << "calculated x_{k+1}" << std::endl;
    }
    /* save a copy of p_rerror_covariance for kalman gain calculation */
    //    std::vector<std::vector<double>> previous_p_error_covariance =
    //        p_error_covariance;
    /*
     * P_{k+1|k+1} = P_{k+1|k} - K_k*C*P_{k+1|k}
     * */
    /* K*C */
    std::vector<std::vector<double>> KC = matrix::multiply(k_kalman_gain, C);
    /* K*C*P_{k+1|k} */
    std::vector<std::vector<double>> KCP =
        matrix::multiply(KC, p_error_covariance);
    /* P_{k+1|k+1} = P_{k+1|k} - K*C*P_{k+1|k} */
    p_error_covariance = matrix::subtract(p_error_covariance, KCP);

    if (_debug) {
      std::cout << "calculated P_{k+1|k+1}" << std::endl;
    }
    /*
     * x_{k} = x_{k+1}
     * */
    x_hat_estimated_state = new_x_hat_estimated_state;

    if (_debug) {
      std::cout << "updated x_k" << std::endl;
    }

    /*
     * K = P_{k+1|k}*transpose(C) * inverse(C*P_{k+1|k}*transpose(C) + R)
     * */
    /* C*P */
    std::vector<std::vector<double>> CP =
        matrix::multiply(C, /*previous_*/ p_error_covariance);
    /* transpose(C) */
    std::vector<std::vector<double>> CT = matrix::transpose(C);
    /* C*P*transpose(C) */
    std::vector<std::vector<double>> CPCT = matrix::multiply(CP, CT);
    /* C*P*transpose(C) + R */
    std::vector<std::vector<double>> CPCT_R =
        matrix::add(CPCT, r_noise_covariance);
    /* inverse(C*P*transpose(C) + R) */
    std::vector<std::vector<double>> CPCT_R_inverse = matrix::inverse(CPCT_R);
    /* P_{k+1|k}*transpose(C) */
    CT = matrix::transpose(C);
    std::vector<std::vector<double>> PCT =
        matrix::multiply(/*previous_*/ p_error_covariance, CT);
    /* K = P_{k+1|k}*transpose(C) * inverse(C*P*transpose(C) + R) */
    k_kalman_gain = matrix::multiply(PCT, CPCT_R_inverse);

    if (_debug) {
      std::cout << "calculated K" << std::endl;
    }

    timestep++;
    return;
  }
};

/*******************************************************************************
 *******************************************************************************
 * LOCALIZATION CLASS
 *******************************************************************************
 ******************************************************************************/
class localization {
private:
  cv::Point2f position = {0, 0};
  float previous_velocity = 0;
  float previous_distance = 0;
  float previous_rotational_velocity = 0;
  float distance_traveled = 0;
  float rotation = 0;
  const float MAX_ROTATION = PI;
  const float MIN_ROTATION = -PI;
  const float TOTAL_ROTATION = 2 * PI;

public:
  kalman_filter *kalman;
  float max_error = 0.0f;

  /*****************************************************************************
   * CONSTRUCTOR
   ****************************************************************************/
  localization(cv::Point _x_start_position = {0, 0})
      : position(_x_start_position) {
    //    const double h_measurement_map_scalar = 1.0f;

    std::vector<std::vector<double>> r = {
        {40.0f, 0, 0}, {0, 40.0f, 0}, {0, 0, 40.0f}};
    std::vector<std::vector<double>> q_estimated_covariance = {
        {10, 0, 0}, {0, 10, 0}, {0, 0, 10}};
    std::vector<std::vector<double>> p_0 = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    std::vector<std::vector<double>> k_0 = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    std::vector<std::vector<double>> x_0 = {{0}, {0}, {0}};

    kalman = new kalman_filter(q_estimated_covariance, p_0, x_0, k_0, r);
    srand(time(NULL));
  };

  /*****************************************************************************
   * CALCULATE DEAD RECKONING
   ****************************************************************************/
  void update_dead_reckoning(float _velocity, float _rotational_velocity,
                             int _debug = NO_DEBUG) {
    /*update distance*/
    previous_distance = distance_traveled;

    /*integrate distance and rotation*/
    distance_traveled = previous_velocity + _velocity;
    rotation = previous_rotational_velocity + _rotational_velocity;

    /*confine rotation to +PI <-> -PI*/
    while (rotation > MAX_ROTATION) {
      rotation -= TOTAL_ROTATION;
    }
    while (rotation < MIN_ROTATION) {
      rotation += TOTAL_ROTATION;
    }
    /*update values for next time step calculation*/
    previous_velocity = distance_traveled;
    previous_rotational_velocity = rotation;

    /*calculate next position*/
    cv::Point2f rotation_and_translation_vector = {
        cos(rotation) * (distance_traveled - previous_distance),
        sin(rotation) * (distance_traveled - previous_distance)};

    position = {position.x + rotation_and_translation_vector.x,
                position.y + rotation_and_translation_vector.y};

    if (_debug) {
      std::cout << std::endl
                << "dr " << position << " " << rotation << std::endl;
    }
    std::vector<std::vector<double>> u_input = {
        {_velocity * cos(_rotational_velocity)},
        {_velocity * (-1) * sin(_rotational_velocity)},
        {_rotational_velocity}};

    std::vector<std::vector<double>> y_output = {
        {position.x}, {position.y}, {rotation}};

    /*random error calculation*/
    float error_1 = 0;
    float error_2 = 0;
    float error_3 = 0;

    if (max_error != 0) {
      error_1 =
          (static_cast<float>(rand()) /
           (static_cast<float>(static_cast<float>(RAND_MAX) / max_error * 2))) -
          max_error;
      error_2 =
          (static_cast<float>(rand()) /
           (static_cast<float>(static_cast<float>(RAND_MAX) / max_error * 2))) -
          max_error;
      error_3 =
          (static_cast<float>(rand()) /
           (static_cast<float>(static_cast<float>(RAND_MAX) / max_error * 2))) -
          max_error;
    }
    std::vector<std::vector<double>> y_noise_output = {
        {y_output[0][0] + error_1},
        {y_output[0][1] + error_2},
        {y_output[0][2] + error_3}};

    kalman->calculate_kalman(u_input, y_noise_output);

    if (_debug) {
      std::cout << "ka [" << kalman->x_hat_estimated_state[0][0] << ","
                << kalman->x_hat_estimated_state[1][0] << "] "
                << kalman->x_hat_estimated_state[2][0] << std::endl;
    }
    return;
  }
  /*****************************************************************************
   * RETURN X std::vector FOR DEBUGGING
   ****************************************************************************/
  cv::Point2f get_position(void) { return position; }
};
