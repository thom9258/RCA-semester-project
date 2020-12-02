#pragma once
#include <algorithm>
#include <fstream>
#include <iostream>
#include <math.h>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <vector>
#include <random>
#include <bitset>
#include <map>
#include <iterator>

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

#define AMOUNTOFACTIONS 4
#define ROOM_AMOUNT 20
enum OPTIONS { NONE = 0, _DEBUG = 1, WAIT = 2, GREYSCALE = 3 };
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
  int room_amount = 0;
  std::vector<std::vector<float>> V = {}; /*Current estimate of state values*/
  std::vector<std::vector<float>> R;      /*Rewards*/
  std::vector<std::vector<std::vector<float>>> Q;      /*Policy*/
    std::map<std::bitset<ROOM_AMOUNT>, float> Q_Markov;
  struct state {
    int x;
    int y;
    bool is_outside_environment; //default: = false;
  };
  // A convenient definition of the terminal state
  const state TERMINAL_STATE = {-1, -1, true};
  float discount_rate = 0.9;
  // threshold for determining the accuracy of the estimation
  float theta = 0.01;
  float epsilon;
  enum action { UP, DOWN, LEFT, RIGHT };

  std::vector<std::vector<int>> room_locations;

  /*****************************************************************************
   * CONSTRUCTOR
   * **************************************************************************/
  QLearning(){};
  QLearning(std::vector<std::vector<int>> _input_map, float _epsilon)
      : environment(_input_map), rows(_input_map.size()),
        columns(_input_map[0].size()), epsilon(_epsilon) {
    for (int x = 0; x < columns; x++) {
      std::vector<float> column;
      for (int y = 0; y < rows; y++) {
        column.push_back(0.0f);
      }
      V.push_back(column);
      R.push_back(column);
    }

    /* Initialise Q matrix */
    for (int x = 0; x < columns; x++) {
      std::vector<std::vector<float>> column;
      for (int y = 0; y < rows; y++) {
          std::vector<float> row;
          for (int z = 0; z < AMOUNTOFACTIONS; z++) {
            row.push_back(0.0f);
          }
          column.push_back(row);
      }
      Q.push_back(column);
    }

    /* Seed random */
    srand(time(NULL));

    /* Index rooms */
    for (int x = 0; x < columns; x++) {
      for (int y = 0; y < rows; y++) {
          if(environment[x][y] != '#'){
              room_locations.push_back({x,y});
              room_amount++;
          }
      }
    }
  }
  /*****************************************************************************
   * GET ROOM INDEX FROM COORDINATES
   * **************************************************************************/
  int get_room_index(int x, int y){
      for(size_t i = 0; i < room_locations.size();i++){
          if(x == room_locations[i][0] && y == room_locations[i][1]){
              return i;

          }
      }
      //This shouldnt happen, no corresponding room
      //std::cout << "Room disaster" << std::endl;
      return -1;
  }

  /*****************************************************************************
   * GET NEXT STATE GIVEN A CURRENT STATE S AND AN ACTION A
   * **************************************************************************/
  state get_next_state(state s, action a) {
    if (s.x < 0 || s.y < 0)
      return TERMINAL_STATE;
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
   * GET BEST ACTION GIVEN A STATE - EPSILON GREEDY
   * **************************************************************************/
  action get_next_action(state s) {
    std::vector<action> possible_actions = {UP, DOWN, LEFT, RIGHT};

    float greedyness = (static_cast<float>(rand()) /
                  (static_cast<float>(static_cast<float>(RAND_MAX))));

    float current_max_value = -1;//std::numeric_limits<float>::min();
    action best_action =
        possible_actions[0]; // Make sure that we have a default action (not
                             // really necessary)

    // Either select action with highest Q(s,a) value, or pick a random action (epsilon-greedy)
    if(greedyness < epsilon){
        //Iterate possible actions, and find the highest Q(s,a) value
        for (size_t i = 0; i < possible_actions.size(); i++) {
          state next = get_next_state(s, possible_actions[i]);
          if (!next.is_outside_environment) {
            float q_value = Q[s.x][s.y][i];
            if (q_value > current_max_value) {
              best_action = possible_actions[i];
              current_max_value = q_value;
            }
          }
        }
        return best_action;
    } else {
        int random_action = rand() % possible_actions.size();
        return possible_actions[random_action];
    }
  }
    
    action get_next_action_markov(state s, std::bitset<ROOM_AMOUNT> key) {
      std::vector<action> possible_actions = {UP, DOWN, LEFT, RIGHT};

      float greedyness = (static_cast<float>(rand()) /
                    (static_cast<float>(static_cast<float>(RAND_MAX))));

      float current_max_value = -1;//std::numeric_limits<float>::min();
      action best_action =
          possible_actions[0]; // Make sure that we have a default action (not
                               // really necessary)

      // Either select action with highest Q(s,a) value, or pick a random action (epsilon-greedy)
      if(greedyness < epsilon){
          //Iterate possible actions, and find the highest Q(s,a) value
          for (size_t i = 0; i < possible_actions.size(); i++) {
            state next = get_next_state(s, possible_actions[i]);
            if (!next.is_outside_environment) {
              float q_value = Q[s.x][s.y][i];
              if (q_value > current_max_value) {
                best_action = possible_actions[i];
                current_max_value = q_value;
              }
            }
          }
          return best_action;
      } else {
          int random_action = rand() % possible_actions.size();
          return possible_actions[random_action];
      }
    }

  /*****************************************************************************
   * GET BEST ACTION GIVEN A STATE - NOT! EPSILON GREEDY
   * **************************************************************************/
  action get_best_action(state s) {
    std::vector<action> possible_actions = {UP, DOWN, LEFT, RIGHT};

    float current_max_value = -1;//std::numeric_limits<float>::min();
    action best_action =
        possible_actions[0]; // Make sure that we have a default action (not
                             // really necessary)

    //Iterate possible actions, and find the highest Q(s,a) value
    for (size_t i = 0; i < possible_actions.size(); i++) {
      state next = get_next_state(s, possible_actions[i]);
      if (!next.is_outside_environment) {
        float q_value = Q[s.x][s.y][i];
        if (q_value > current_max_value) {
          best_action = possible_actions[i];
          current_max_value = q_value;
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
   * PRINT ROOM INDEXES
   * **************************************************************************/
  void print_rooms() {
    for (int x = 0; x < columns; x++) {
      for (int y = 0; y < rows; y++) {
        int room_index = get_room_index(x,y);
        if (room_index != -1) {
          printf("%2.2i", room_index);
          //std::cout << room_index;
        } else {
          //std::cout << '#';
          printf("--");
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
    for (int x = 0; x < columns; x++) {
      for (int y = 0; y < rows; y++){
          if(environment[x][y] != '#'){
              printf(" %5.2f ", V[x][y]);
          } else{
              printf("       ");
          }

      }
      printf("\n");
    }
  }

  /*****************************************************************************
   * PRINT POLICY
   * **************************************************************************/
  void print_policy() {
    std::cout << "Print policy:" << std::endl;
    for (int y = 0; y < rows; y++) {
      for (int x = 0; x < columns; x++) {
        if (environment[y][x] != '#' && environment[y][x] != 'd' &&
            environment[y][x] != '2') {
          action a = get_best_action((state){
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
