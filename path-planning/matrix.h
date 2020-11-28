#pragma once
#include "localization.h"
#include <cmath>
#include <iostream>
#include <path_planning.h>
#include <stdexcept>
#include <stdio.h>
#include <vector>
/*
 * Description:     A matrix class implementation working in
 *                  std::vector<std::vector<double>>
 *
 * Creation date:   271120
 */

/*******************************************************************************
 *******************************************************************************
 * MATRIX CLASS
 *******************************************************************************
 ******************************************************************************/
class matrix {
public:
  std::vector<std::vector<double>> data;

  /*****************************************************************************
   * CONSTRUCTOR
   ****************************************************************************/
  matrix(std::vector<std::vector<double>> _input) : data(_input){};

  /*****************************************************************************
   * GET DETERMINANT OF MATRICES
   ****************************************************************************/
  static double determinant(std::vector<std::vector<double>> vect) {
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
      result = result + sign * vect[0][i] * determinant(subVect);
      sign = -sign;
    }

    return result;
  }

  /*****************************************************************************
   * GET COFACTOR OF MATRICES
   ****************************************************************************/
  static std::vector<std::vector<double>>
  cofactor(std::vector<std::vector<double>> vect) {
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
        solution[i][j] = pow(-1, i + j) * determinant(subVect);
      }
    }
    return solution;
  }

  /*****************************************************************************
   * INVERSE MATRICES
   ****************************************************************************/
  static std::vector<std::vector<double>>
  inverse(std::vector<std::vector<double>> vect) {
    if (determinant(vect) == 0) {
      throw std::runtime_error("Determinant is 0");
    }

    double d = 1.0 / determinant(vect);
    std::vector<std::vector<double>> solution(vect.size(),
                                              std::vector<double>(vect.size()));

    for (size_t i = 0; i < vect.size(); i++) {
      for (size_t j = 0; j < vect.size(); j++) {
        solution[i][j] = vect[i][j];
      }
    }

    solution = transpose(cofactor(solution));

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
  static std::vector<std::vector<double>>
  transpose(std::vector<std::vector<double>> matrix1) {

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
  static std::vector<std::vector<double>>
  multiply(std::vector<std::vector<double>> a,
           std::vector<std::vector<double>> b, int _debug = NO_DEBUG) {
    if (_debug) {
      std::cout << "multiply start" << std::endl;
    }
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
    if (_debug) {
      std::cout << "multiply end" << std::endl;
    }
    return c;
  }

  /*****************************************************************************
   * ADD MATRICES
   ****************************************************************************/
  static std::vector<std::vector<double>>
  add(std::vector<std::vector<double>> A, std::vector<std::vector<double>> B,
      int _debug = NO_DEBUG) {
    if (_debug) {
      std::cout << "add start" << std::endl;
    }
    std::vector<std::vector<double>> C(A.size());

    for (size_t i = 0; i < A.size(); i++) {
      C[i].resize(A[i].size());
      for (size_t j = 0; j < A[i].size(); j++) {
        C[i][j] = A[i][j] + B[i][j];
        if (_debug) {
          std::cout << "changed" << i << " " << j << " to " << A[i][j] + B[i][j]
                    << std::endl;
        }
      }
    }

    return C;
  }

  /*****************************************************************************
   * SUBTRACT MATRICES
   ****************************************************************************/
  static std::vector<std::vector<double>>
  subtract(std::vector<std::vector<double>> A,
           std::vector<std::vector<double>> B) {
    std::vector<std::vector<double>> C(A.size());

    for (size_t i = 0; i < A.size(); i++) {
      C[i].resize(A[i].size());
      for (size_t j = 0; j < A[i].size(); j++) {
        C[i][j] = A[i][j] - B[i][j];
      }
    }
    return C;
  }
  /*****************************************************************************
   * PRINT MATRICES
   ****************************************************************************/
  static void print(std::vector<std::vector<double>> _input,
                    std::string _name = "") {
    if (_name != "") {
      std::cout << _name << std::endl;
    }
    for (size_t i = 0; i < _input.size(); i++) {
      for (size_t j = 0; j < _input[0].size(); j++) {
        std::cout << _input[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }
};
