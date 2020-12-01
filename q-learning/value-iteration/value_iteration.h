#pragma once
#include <algorithm>
#include <fstream>
#include <iostream>
#include <math.h>
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
 * Changelog:       DDMMYY  XX        Change
 *                  311120  TH/MI/ER  Started q-learning class
 *                  011220  TH/MI/ER  Expanded q-learning class with
 *
 */

enum OPTIONS { NONE = 0, DEBUG = 1, WAIT = 2, GREYSCALE = 3 };
enum POSITION { X = 0, Y = 1 };

class QLearning {
private:
  /*****************************************************************************
   * CHECK IF A POSITION IS VALID INSIDE MAP
   * **************************************************************************/
  int on_board(std::vector<int> next_position,
               std::vector<std::vector<int>> _environment) {
    if (next_position[X] < 0 || next_position[X] > columns ||
        next_position[Y] < 0 || next_position[Y] > rows ||
        _environment[next_position[X]][next_position[Y]] == '#') {
      return false;
    }
    return true;
  }

public:
  // Environment -- spaces: agent can move, "+": reward, "-": punishment.
  std::vector<std::vector<int>> environment = {};
  int rows, columns;
  std::vector<std::vector<float>> V = {}; /*Current estimate of state values*/
  std::vector<std::vector<float>> R;      /*Rewards*/
  std::vector<std::vector<float>> Q;      /*Policy*/
  struct state {
    int x;
    int y;
    bool is_outside_environment = false;
  };
  // A convenient definition of the terminal state
  const state TERMINAL_STATE = {-1, -1, true};
  float discount_rate = 0.9;
  // threshold for determining the accuracy of the estimation
  float theta = 0.01;
  enum action { UP, DOWN, LEFT, RIGHT };

  /*****************************************************************************
   * CONSTRUCTOR
   * **************************************************************************/
  QLearning(){};
  QLearning(std::vector<std::vector<int>> _input_image)
      : environment(_input_image), rows(_input_image.size()),
        columns(_input_image[0].size()) {
    for (int x = 0; x < columns; x++) {
      std::vector<float> column;
      for (int y = 0; y < rows; y++) {
        column.push_back(0.0f);
      }
      V.push_back(column);
      R.push_back(column);
      Q.push_back(column);
    }
  }

  /*****************************************************************************
   * GET NEXT STATE GIVEN A CURRENT STATE S AND AN ACTION A
   * **************************************************************************/
  state get_next_state(state s, action a) {
    if (environment[s.y][s.x] == '#' || environment[s.y][s.x] == '2' /*cake*/ ||
        environment[s.y][s.x] == 'd' /*trap*/)
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

  /*****************************************************************************
   * GET REWARD GIVEN A STATE AND AN ACTION
   * **************************************************************************/
  float get_reward(state s, action a) {
    state next = get_next_state(s, a);
    if (next.is_outside_environment) {
      return 0;
    } else {
      if (environment[next.y][next.x] == '2' /*cake*/)
        return 1.0;

      if (environment[next.y][next.x] == 'd' /*trap*/)
        return -1.0;

      return 0;
    }
  }

  /*****************************************************************************
   * GET REWARD GIVEN A STATE AND AN ACTION
   * **************************************************************************/
  action get_next_action(state s) {
    std::vector<action> possible_actions = {UP, DOWN, LEFT, RIGHT};

    float current_max_value = std::numeric_limits<float>::min();
    action best_action =
        possible_actions[0]; // Make sure that we have a default action (not
                             // really necessary)

    for (const auto a : possible_actions) {
      state next = get_next_state(s, a);
      float reward = get_reward(s, a);
      if (!next.is_outside_environment) {
        if (V[next.y][next.x] + reward > current_max_value) {
          best_action = a;
          current_max_value = V[next.y][next.x] + reward;
        }
      }
    }
    return best_action;
  }

  /*****************************************************************************
   * PRINT THE ENVIRONMENT
   * **************************************************************************/
  void print_environment() {
    for (int x = 0; x < columns; x++) {
      for (int y = 0; y < rows; y++) {
        if (environment[x][y] == '#' || environment[x][y] == 's') {
          std::cout << char(environment[x][y]);
        } else {
          std::cout << '-';
        }
        std::cout << ' ';
      }
      std::cout << std::endl;
    }
  }

  /*****************************************************************************
   * PRINT THE CURRENT ESTIMATE OF STATE VALUES
   * **************************************************************************/
  void print_state_values() {
    for (int y = 0; y < rows; y++) {
      for (int x = 0; x < columns; x++)
        printf(" %5.2f ", V[y][x]);

      printf("\n");
    }
  }

  /*****************************************************************************
   * PRINT POLICY
   * **************************************************************************/
  void print_policy() {
    for (int y = 0; y < rows; y++) {
      for (int x = 0; x < columns; x++) {
        if (environment[y][x] != '#' && environment[y][x] != 'd' &&
            environment[y][x] != '2') {
          action a = get_next_action((state){
              x,
              y,
          });

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
            /*FALLTHROUGH*/
          }
        } else {
          printf("   %c   ", environment[y][x]);
        }
      }

      printf("\n");
    }
  }
};
