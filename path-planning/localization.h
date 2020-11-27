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

  /*****************************************************************************
   * GET COFACTOR OF MATRICES
   ****************************************************************************/
  std::vector<std::vector<double>>
  getCofactor(const std::vector<std::vector<double>> vect) {
    if (vect.size() != vect[0].size()) {
      throw std::runtime_error("Matrix is not quadratic");
    }

    std::vector<std::vector<double>> solution(vect.size(),
                                              std::vector<double>(vect.size()));
    std::vector<std::vector<double>> subVect(
        vect.size() - 1, std::vector<double>(vect.size() - 1));

    for (std::size_t i = 0; i < vect.size(); i++) {
      for (std::size_t j = 0; j < vect[0].size(); j++) {

        int p = 0;
        for (size_t x = 0; x < vect.size(); x++) {
          if (x == i) {
            continue;
          }
          int q = 0;

          for (size_t y = 0; y < vect.size(); y++) {
            if (y == j) {
              continue;
            }

            subVect[p][q] = vect[x][y];
            q++;
          }
          p++;
        }
        solution[i][j] = pow(-1, i + j) * getDeterminant(subVect);
      }
    }
    return solution;
  }

  /*****************************************************************************
   * GET DETERMINANT OF MATRICES
   ****************************************************************************/
  double getDeterminant(const std::vector<std::vector<double>> vect) {
    if (vect.size() != vect[0].size()) {
      throw std::runtime_error("Matrix is not quadratic");
    }
    int dimension = vect.size();

    if (dimension == 0) {
      return 1;
    }

    if (dimension == 1) {
      return vect[0][0];
    }

    // Formula for 2x2-matrix
    if (dimension == 2) {
      return vect[0][0] * vect[1][1] - vect[0][1] * vect[1][0];
    }

    double result = 0;
    int sign = 1;
    for (int i = 0; i < dimension; i++) {

      // Submatrix
      std::vector<std::vector<double>> subVect(
          dimension - 1, std::vector<double>(dimension - 1));
      for (int m = 1; m < dimension; m++) {
        int z = 0;
        for (int n = 0; n < dimension; n++) {
          if (n != i) {
            subVect[m - 1][z] = vect[m][n];
            z++;
          }
        }
      }

      // recursive call
      result = result + sign * vect[0][i] * getDeterminant(subVect);
      sign = -sign;
    }

    return result;
  }

  /*****************************************************************************
   * INVERSE MATRICES
   ****************************************************************************/
  std::vector<std::vector<double>>
  getInverse(const std::vector<std::vector<double>> vect) {
    if (getDeterminant(vect) == 0) {
      throw std::runtime_error("Determinant is 0");
    }

    double d = 1.0 / getDeterminant(vect);
    std::vector<std::vector<double>> solution(vect.size(),
                                              std::vector<double>(vect.size()));

    for (size_t i = 0; i < vect.size(); i++) {
      for (size_t j = 0; j < vect.size(); j++) {
        solution[i][j] = vect[i][j];
      }
    }

    solution = getTranspose(getCofactor(solution));

    for (size_t i = 0; i < vect.size(); i++) {
      for (size_t j = 0; j < vect.size(); j++) {
        solution[i][j] *= d;
      }
    }

    return solution;
  }

  /*****************************************************************************
   * TRANSPOSE MATRICES
   ****************************************************************************/
  std::vector<std::vector<double>>
  getTranspose(std::vector<std::vector<double>> matrix1) {

    // Transpose-matrix: height = width(matrix), width = height(matrix)
    std::vector<std::vector<double>> solution(
        matrix1[0].size(), std::vector<double>(matrix1.size()));

    // Filling solution-matrix
    for (size_t i = 0; i < matrix1.size(); i++) {
      for (size_t j = 0; j < matrix1[0].size(); j++) {
        solution[j][i] = matrix1[i][j];
      }
    }
    return solution;
  }

  /*****************************************************************************
   * MULTIPLY MATRICES
   ****************************************************************************/
  std::vector<std::vector<double>>
  Multiply(std::vector<std::vector<double>> &a,
           std::vector<std::vector<double>> &b) {
    const int n = a.size();
    const int m = a[0].size();
    const int p = b[0].size();

    std::vector<std::vector<double>> c(n, std::vector<double>(p, 0));
    for (auto j = 0; j < p; ++j) {
      for (auto k = 0; k < m; ++k) {
        for (auto i = 0; i < n; ++i) {
          c[i][j] += a[i][k] * b[k][j];
        }
      }
    }
    return c;
  }

  /*****************************************************************************
   * ADD MATRICES
   ****************************************************************************/
  std::vector<std::vector<double>>
  MatrixAdder(const std::vector<std::vector<double>> &A,
              const std::vector<std::vector<double>> &B) {
    std::vector<std::vector<double>> C(A.size());

    for (size_t i = 0; i < A.size(); i++) {
      C[i].resize(A[i].size());
      for (size_t j = 0; j < A[i].size(); j++) {
        C[i][j] = A[i][j] + B[i][j];
      }
    }

    return C;
  }

  /*****************************************************************************
   * SUBTRACT MATRICES
   ****************************************************************************/
  std::vector<std::vector<double>>
  matrixSubtractor(const std::vector<std::vector<double>> &A,
                   const std::vector<std::vector<double>> &B) {
    std::vector<std::vector<double>> C(A.size());

    for (size_t i = 0; i < A.size(); i++) {
      C[i].resize(A[i].size());
      for (size_t j = 0; j < A[i].size(); j++) {
        C[i][j] = A[i][j] - B[i][j];
      }
    }

    return C;
  }

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
    new_x_hat_estimated_state = MatrixAdder(
        (Multiply(A, x_hat_estimated_state)), (Multiply(B, _u_input)));

    /*
     * P_{k+1|k} = A*P_{k|k}*transpose(A) + Q
     * */
    std::vector<std::vector<double>> AP = Multiply(A, p_error_covariance);
    std::vector<std::vector<double>> AT = getTranspose(A);
    p_error_covariance =
        MatrixAdder((Multiply(AP, AT)), q_estimated_covariance);

    /*
     * K = P_{k+1|k}*transpose(C) * inverse(C*P*transpose(C) + R)
     * */
    std::vector<std::vector<double>> CP = Multiply(C, p_error_covariance);
    std::vector<std::vector<double>> CT = getTranspose(C);
    std::vector<std::vector<double>> CPCT = Multiply(CP, CT);
    std::vector<std::vector<double>> CPCT_R =
        MatrixAdder(CPCT, r_noise_covariance);
    std::vector<std::vector<double>> CPCT_R_inverse = getInverse(CPCT_R);
    std::vector<std::vector<double>> PCT = Multiply(p_error_covariance, CT);
    k_kalman_gain = Multiply(PCT, CPCT_R_inverse);

    /*Update*/
    /*
     * x_{k+1} = x_{k+1} + K * (y - C * x_{k+1})
     * */
    std::vector<std::vector<double>> Y_CX =
        matrixSubtractor(_y_result, Multiply(C, new_x_hat_estimated_state));
    new_x_hat_estimated_state = MatrixAdder(new_x_hat_estimated_state, Y_CX);

    /*
     * P_{k+1|k+1} = P_{k+1|k} - K*C*P_{k+1|k}
     * */
    std::vector<std::vector<double>> KC = Multiply(k_kalman_gain, C);
    std::vector<std::vector<double>> KCP = Multiply(KC, p_error_covariance);
    p_error_covariance = matrixSubtractor(p_error_covariance, KCP);

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
