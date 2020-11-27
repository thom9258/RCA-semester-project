#pragma once
#include "matrix.h"
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
 *                  271120  TH/MI/ER  Started implementation of kalman filtering
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
  float previous_rotational_velocity = 0;
  float distance_traveled = 0;
  float rotation = 0;
  const float MAX_ROTATION = PI;
  const float MIN_ROTATION = -PI;
  const float TOTAL_ROTATION = 2 * PI;

public:
  localization(cv::Point _x_start_position = {0, 0})
      : x_vector(_x_start_position){};

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

    x_vector = {x_vector.x + rotation_and_translation_vector.x,
                x_vector.y + rotation_and_translation_vector.y};

    if (_debug) {
      std::cout << "velocity: " << _velocity
                << " direction: " << _rotational_velocity << std::endl
                << x_vector << " " << rotation;
    }
    return;
  }
  /*****************************************************************************
   * RETURN X std::vector FOR DEBUGGING
   ****************************************************************************/
  cv::Point2f get_x_vector(void) { return x_vector; }
};

/*******************************************************************************
 *******************************************************************************
 * KALMAN FILTER CLASS
 *******************************************************************************
 ******************************************************************************/
class kalman_filter {
private:
  const double h_measurement_map_scalar = 1.0f;
  unsigned long long timestep = 0;

  std::vector<std::vector<double>> r_noise_covariance = {
      {40.0f, 0, 0}, {0, 40.0f, 0}, {0, 0, 40.0f}};
  std::vector<std::vector<double>> q_estimated_covariance = {
      {10, 0, 0}, {0, 10, 0}, {0, 0, 10}};
  std::vector<std::vector<double>> p_error_covariance = {
      {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  std::vector<std::vector<double>> k_kalman_gain = {
      {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  std::vector<std::vector<double>> x_hat_estimated_state = {};     /*x_k*/
  std::vector<std::vector<double>> new_x_hat_estimated_state = {}; /*x_{k+1}*/
  std::vector<std::vector<double>> A = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  std::vector<std::vector<double>> B = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  std::vector<std::vector<double>> C = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  std::vector<std::vector<double>> D = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

public:
  /*****************************************************************************
   * CONSTRUCTOR
   ****************************************************************************/
  kalman_filter(std::vector<std::vector<double>> _q,
                std::vector<std::vector<double>> _p,
                std::vector<std::vector<double>> _x, double _k)
      : q_estimated_covariance(_q), p_error_covariance(_p), k_kalman_gain(_k),
        x_hat_estimated_state(_x), new_x_hat_estimated_state(_x){};

  /*****************************************************************************
   * UPDATE KALMAN FILTER
   ****************************************************************************/
  void calculate_kalman(std::vector<std::vector<double>> _u_input,
                        std::vector<std::vector<double>> _y_result) {

    /*Prediction*/
    /*
     * x_{k+1} = A * x_k + B*u
     * */
    new_x_hat_estimated_state = matrix::MatrixAdder(
        (matrix::matrix::Multiply(A, x_hat_estimated_state)),
        (matrix::Multiply(B, _u_input)));

    /*
     * P_{k+1|k} = A*P_{k|k}*transpose(A) + Q
     * */
    std::vector<std::vector<double>> AP =
        matrix::Multiply(A, p_error_covariance);
    std::vector<std::vector<double>> AT = matrix::getTranspose(A);
    p_error_covariance =
        matrix::MatrixAdder((matrix::Multiply(AP, AT)), q_estimated_covariance);

    /*
     * K = P_{k+1|k}*transpose(C) * inverse(C*P*transpose(C) + R)
     * */
    std::vector<std::vector<double>> CP =
        matrix::Multiply(C, p_error_covariance);
    std::vector<std::vector<double>> CT = matrix::getTranspose(C);
    std::vector<std::vector<double>> CPCT = matrix::Multiply(CP, CT);
    std::vector<std::vector<double>> CPCT_R =
        matrix::MatrixAdder(CPCT, r_noise_covariance);
    std::vector<std::vector<double>> CPCT_R_inverse =
        matrix::getInverse(CPCT_R);
    std::vector<std::vector<double>> PCT =
        matrix::Multiply(p_error_covariance, CT);
    k_kalman_gain = matrix::Multiply(PCT, CPCT_R_inverse);

    /*Update*/
    /*
     * x_{k+1} = x_{k+1} + K * (y - C * x_{k+1})
     * */
    std::vector<std::vector<double>> Y_CX = matrix::matrixSubtractor(
        _y_result, matrix::Multiply(C, new_x_hat_estimated_state));
    new_x_hat_estimated_state =
        matrix::MatrixAdder(new_x_hat_estimated_state, Y_CX);

    /*
     * P_{k+1|k+1} = P_{k+1|k} - K*C*P_{k+1|k}
     * */
    std::vector<std::vector<double>> KC = matrix::Multiply(k_kalman_gain, C);
    std::vector<std::vector<double>> KCP =
        matrix::Multiply(KC, p_error_covariance);
    p_error_covariance = matrix::matrixSubtractor(p_error_covariance, KCP);

    /*
     * updates x_{k}:
     *
     * x_{k} = x_{k+1}
     * */
    x_hat_estimated_state = new_x_hat_estimated_state;

    timestep++;
    return;
  }
};
