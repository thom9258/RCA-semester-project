#pragma once
#include <algorithm>
#include <fstream>
#include <iostream>
#include <math.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <vector>

/*
 * Description:     A path planning class implementation for the project
 *                  RCA 5.semester.
 *
 * Creation date:   281020
 *
 * Changelog:       DDMMYY  XX  Change
 *                  311120  TH/MI/ER  Started q-learning class
 *
 */

enum DODEBUG { NO_DEBUG = 0, NODEBUG = 0, DEBUG = 1 };
enum DOWAIT { NO_WAIT = 0, NOWAIT = 0, WAIT = 1 };
enum WHATCOLOR { GREYSCALE = 0, COLOR = 1 };
#define GREEN                                                                  \
  { 0, 255, 0 }
#define BLUE                                                                   \
  { 255, 0, 0 }
#define RED                                                                    \
  { 0, 0, 255 }
#define WHITE                                                                  \
  { 255, 255, 255 }
#define BLACK                                                                  \
  { 0, 0, 0 }
#define GREY                                                                   \
  { 169, 169, 169 }

const cv::Vec3b green_pixel = GREEN;
const cv::Vec3b blue_pixel = BLUE;
const cv::Vec3b red_pixel = RED;
const cv::Vec3b white_pixel = WHITE;
const cv::Vec3b black_pixel = BLACK;
const cv::Vec3b grey_pixel = GREY;

class QLearning {
private:
  //----------------------------------------------------------------------------
  // GET PIXEL VALUE ARRANGED IN [BLUE GREEN RED] ORDER
  //----------------------------------------------------------------------------
  cv::Vec3b get_pixel_color(cv::Point _point, cv::Mat _map) {
    /*white = [255, 255, 255], black = [0, 0, 0]*/
    return _map.at<cv::Vec3b>(_point);
  }

  //----------------------------------------------------------------------------
  // CHECK IF A POSITION IS VALID INSIDE MAP
  //----------------------------------------------------------------------------
  bool on_board(cv::Point next_position) {
    if (next_position.x < 0 || next_position.x > rows || next_position.y < 0 ||
        next_position.y > columns) {
      return true;
    }
    return false;
  }

public:
  QLearning(){};
  cv::Mat input_image;
  // Environment -- spaces: agent can move, "+": reward, "-": punishment.
  std::vector<std::vector<char>> environment = {};
  int rows, columns;
  std::vector<std::vector<float>> V; /*Current estimate of state values*/
  std::vector<std::vector<float>> R; /*Rewards*/
  std::vector<std::vector<float>> Q; /*Policy*/

  QLearning(cv::Mat _input_image)
      : input_image(_input_image), rows(_input_image.rows),
        columns(_input_image.cols) {
    /*Setup estimate map*/
    for (int i = 0; i < rows; i++) {
      std::vector<float> row;
      for (int j = 0; j < columns; j++) {
        row.push_back(0.0f);
      }
      R.push_back(row);
      V.push_back(row);
      Q.push_back(row);
    }

    /*Setup obstacle map*/
    for (int i = 0; i < columns; i++) {
      std::vector<char> column;
      for (int j = 0; j < rows; j++) {
        if (get_pixel_color({i, j}, _input_image) == black_pixel) {
          column.push_back('#');
        } else {
          column.push_back(' ');
        }
      }
      environment.push_back(column);
    }
  }

  // Current estimate of state values under the current policy:

  // State is given by (x, y) in the environment. Must be inside the
  // environment to be valid
  struct state {
    int x;
    int y;
    bool is_outside_environment;
  };

  // A convenient definition of the terminal state
  const state TERMINAL_STATE = {-1, -1, true};

  // Discount rate:
  float discount_rate = 0.9;

  // Theta: the thredhold for determining the accuracy of the estimation
  float theta = 0.01;

  // Actions:
  enum action { UP, DOWN, LEFT, RIGHT };

  // Get the next state given a current state s and an action a:
  state GetNextState(state s, action a) {
    if (environment[s.y][s.x] != ' ')
      return TERMINAL_STATE;

    switch (a) {
    case UP:
      s.y -= 1;
      break;
    case DOWN:
      s.y += 1;
      break;
    case LEFT:
      s.x -= 1;
      break;
    case RIGHT:
      s.x += 1;
      break;
    }

    if (s.x < 0 || s.y < 0 || s.x >= columns || s.y >= rows)
      return TERMINAL_STATE;

    s.is_outside_environment = false;
    return s;
  }

  // Ger the reward given a state and an action:
  float GetReward(state s, action a) {
    state next = GetNextState(s, a);
    if (next.is_outside_environment) {
      return 0;
    } else {
      if (environment[next.y][next.x] == '+')
        return 1.0;

      if (environment[next.y][next.x] == '-')
        return -1.0;

      return 0;
    }
  }

  // Get the best action computed according to the current state-value
  // estimate:
  action GetNextAction(state s) {
    std::vector<action> possible_actions = {UP, DOWN, LEFT, RIGHT};

    float current_max_value = std::numeric_limits<float>::min();
    action best_action =
        possible_actions[0]; // Make sure that we have a default action (not
                             // really necessary)

    for (const auto a : possible_actions) {
      state next = GetNextState(s, a);
      float reward = GetReward(s, a);
      if (!next.is_outside_environment) {
        if (V[next.y][next.x] + reward > current_max_value) {
          best_action = a;
          current_max_value = V[next.y][next.x] + reward;
        }
      }
    }
    return best_action;
  }

  // Print the environment with border around:
  void PrintEnvironment() {
    for (int y = -1; y <= rows; y++) {
      for (int x = -1; x <= columns; x++)
        if (y < 0 || y >= rows || x < 0 || x >= columns)
          std::cout << "#";
        else
          std::cout << environment[y][x];

      std::cout << std::endl;
    }
  }

  // Print the current estimate of state values:
  void PrintStateValues() {
    for (int y = 0; y < rows; y++) {
      for (int x = 0; x < columns; x++)
        printf(" %5.2f ", V[y][x]);

      printf("\n");
    }
  }

  // Print the current estimate of state values:
  void PrintPolicy() {
    for (int y = 0; y < rows; y++) {
      for (int x = 0; x < columns; x++) {
        if (environment[y][x] == ' ') {
          action a = GetNextAction((state){x, y});

          switch (a) {
          case UP:
            printf("  UP   ");
            break;
          case DOWN:
            printf(" DOWN  ");
            break;
          case LEFT:
            printf(" LEFT  ");
            break;
          case RIGHT:
            printf(" RIGHT ");
            break;
          default:
            printf(" UNKNOWN ACTION! ");
          }
        } else {
          printf("   %c   ", environment[y][x]);
        }
      }

      printf("\n");
    }
  }
};
