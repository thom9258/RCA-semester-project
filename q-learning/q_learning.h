#pragma once
#include <algorithm>
#include <bitset>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <random>
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
 *                  011220  TH/MI/ER  Expanded q-learning class
 *                  021220  TH/MI/ER  Expanded q-learning class with std::maps
 *                                    (associative array)
 *                  031220  TH/MI/ER  Fixed key for std::map fixed x,y
 *                                    definitions
 *
 */

#define AMOUNTOFACTIONS 4
#define ROOM_AMOUNT 20
#define MIN_FLOAT -999.99f
#define RANDOM_MARBLES 1
enum OPTIONS { NONE = 0, DEBUG = 1, WAIT = 2, GREYSCALE = 3 };
enum POSITION { X = 0, Y = 1 };

class q_learning {
private:
public:
  int start_x;
  int start_y;
  std::vector<std::vector<int>> environment = {};
  std::vector<std::vector<float>> marble_map;

  int rows;    /*The amount of columns (length of a row)*/
  int columns; /*The amount of rows (length of a column)*/
  int room_amount = 0;

  std::map<std::string, float> Q_Markov = {};
  struct state {
    int x;
    int y;
    bool is_outside_environment; // default: = false;
    std::bitset<ROOM_AMOUNT> visited_list;
  };
  const state TERMINAL_STATE = {-1, -1, true, 0};
  float discount_rate = 0.9;
  float epsilon_value;
  bool random_marbles;
  enum action { UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3 };

  std::vector<std::vector<int>> room_locations;
  std::vector<int> marble_locations;

  /*****************************************************************************
   * CONSTRUCTOR
   * **************************************************************************/
  q_learning(){};
  q_learning(std::vector<std::vector<int>> _input_map,
             std::vector<std::vector<float>> _marble_map, float _epsilon_value,
             bool _random_marbles = false)
      : environment(_input_map), marble_map(_marble_map),
        rows(_input_map[0].size()), columns(_input_map.size()),
        epsilon_value(_epsilon_value), random_marbles(_random_marbles) {

    /* Seed random */
    srand(time(NULL));

    /* Index rooms */
    for (int x = 0; x < columns; x++) {
      for (int y = 0; y < rows; y++) {
        if (environment[x][y] != '#') {
          room_locations.push_back({x, y});
          room_amount++;
        }
        if (environment[x][y] == 's' /*start*/) {
          start_x = x;
          start_y = y;
        }
      }
    }
    if (random_marbles == RANDOM_MARBLES) {
      generate_random_marbles();
    }
  }

  /*****************************************************************************
   * GET Q VALUE FROM MARKOV Q TABLE
   * **************************************************************************/
  float get_q_value(int x, int y, int _action,
                    std::bitset<ROOM_AMOUNT> _visited_list) {
    // Generate key
    // 2 bits for action, 5 bits for room, 20 bits from visitedlist

    std::bitset<5> room_index(get_room_index(x, y));
    std::bitset<2> action(_action);
    std::bitset<ROOM_AMOUNT + 7> key(room_index.to_string() +
                                     action.to_string() +
                                     _visited_list.to_string());
    //    std::cout << "key: " << key.to_string() << std::endl;

    // Check if Q value exists for key, if not, initialise it to 0.0f
    if (!Q_Markov.count(key.to_string())) {
      Q_Markov.insert(std::pair<std::string, float>(key.to_string(), 0.0f));
    }
    float result = Q_Markov[key.to_string()];
    //    if (result != 0) {
    //      std::cout << result << std::endl;
    //    }
    return result;
  }

  /*****************************************************************************
   * SET Q VALUE IN MARKOV Q TABLE
   * **************************************************************************/
  void set_q_value(int x, int y, int _action,
                   std::bitset<ROOM_AMOUNT> _visited_list, float _insert) {
    // Generate key
    // 2 bits for action, 5 bits for room, 20 bits from visitedlist
    std::bitset<5> room_index(get_room_index(x, y));
    std::bitset<2> action(_action);
    std::bitset<ROOM_AMOUNT + 7> key(room_index.to_string() +
                                     action.to_string() +
                                     _visited_list.to_string());

    //    std::cout << "key: " << key.to_string() << std::endl;
    // Check if Q value exists for key, if not, initialise it to 0.0f
    if (!Q_Markov.count(key.to_string())) {
      Q_Markov.insert(std::pair<std::string, float>(key.to_string(), _insert));
    } else {
      Q_Markov[key.to_string()] = _insert;
    }
  }

  /*****************************************************************************
   * GET ROOM INDEX FROM COORDINATES
   * **************************************************************************/
  int get_room_index(int x, int y) {
    for (size_t i = 0; i < room_locations.size(); i++) {
      if (x == room_locations[i][0] && y == room_locations[i][1]) {
        return i;
      }
    }
    // This shouldnt happen, no corresponding room
    // std::cout << "Room disaster" << std::endl;
    return -1;
  }

  /*****************************************************************************
   * GET NEXT STATE GIVEN A CURRENT STATE S AND AN ACTION A
   * **************************************************************************/
  state get_next_state(state s, action a) {
    if (s.x < 0 || s.y < 0)
      return TERMINAL_STATE;
    if (environment[s.x][s.y] == '#' ||
        /*environment[s.x][s.y] == 'c'*/ /*marble ||*/
            environment[s.x][s.y] == 'd' /*trap*/)
      return TERMINAL_STATE;

    std::bitset<ROOM_AMOUNT> new_visited_list = s.visited_list;
    new_visited_list.set(get_room_index(s.x, s.y));

    if (s.visited_list.all() /*are set*/) {
      return TERMINAL_STATE;
    }

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

    s.visited_list = new_visited_list;
    s.is_outside_environment = false;
    return s;
  }

  /*****************************************************************************
   * GET REWARD GIVEN A STATE AND AN ACTION
   * **************************************************************************/
  float get_reward(state s, action a) {
    if (s.visited_list.all() /*are set*/) {
      return 1.0f;
    }
    state next = get_next_state(s, a);
    if (next.is_outside_environment) {
      return 0.0f;
    }
    if (environment[next.x][next.y] == 'c' /*marble*/
        && !s.visited_list[get_room_index(next.x, next.y)]) {
      return 1.0f;
    }
    if (environment[next.x][next.y] == 'd' /*trap*/ ||
        environment[next.x][next.y] == '#' /*wall*/) {
      return -1.0f;
    }
    return 0.0f;
  }

  /*****************************************************************************
   * GET NEXT ACTION GIVEN A STATE - epsilon_value GREEDY
   * **************************************************************************/
  action get_next_action(state s) {
    float current_max_value = MIN_FLOAT; // std::numeric_limits<float>::min();
    std::vector<action> possible_actions = {UP, DOWN, LEFT, RIGHT};
    std::vector<action> current_best_actions = {};
    float greedyness = (static_cast<float>(rand()) /
                        (static_cast<float>(static_cast<float>(RAND_MAX))));

    // Either select action with highest Q(s,a) value, or pick a random action
    // (epsilon_value-greedy)
    if (greedyness > epsilon_value) {

      // Iterate possible actions, and find the highest Q(s,a) value
      for (size_t i = 0; i < possible_actions.size(); i++) {
        state next = get_next_state(s, possible_actions[i]);
        if (!next.is_outside_environment) {
          float q_value = get_q_value(s.x, s.y, i, s.visited_list);
          if (q_value > current_max_value) {
            current_best_actions = {};
            current_best_actions.push_back(possible_actions[i]);
            current_max_value = q_value;
          } else if (q_value == current_max_value) {
            current_best_actions.push_back(possible_actions[i]);
            current_max_value = q_value;
          }
        }
      }
      if (current_best_actions.size() == 1) {
        return current_best_actions[0];
      }
      if (current_best_actions.size() == 0) {
        return possible_actions[0];
      }
      return current_best_actions[(rand() % current_best_actions.size())];
    } else { /*epsilon greedy*/
      return possible_actions[rand() % possible_actions.size()];
    }
  }

  /*****************************************************************************
   * GET BEST ACTION GIVEN A STATE - NOT! epsilon_value GREEDY
   * **************************************************************************/
  action get_best_action(state s, int _option = NONE) {
    float current_max_value = MIN_FLOAT; // std::numeric_limits<float>::min();
    std::vector<action> possible_actions = {UP, DOWN, LEFT, RIGHT};
    std::vector<action> current_best_actions = {};

    // Iterate possible actions, and find the highest Q(s,a) value
    for (size_t i = 0; i < possible_actions.size(); i++) {
      state next = get_next_state(s, possible_actions[i]);
      if (!next.is_outside_environment) {
        float q_value = get_q_value(s.x, s.y, i, s.visited_list);

        if (_option == DEBUG) {
          if (q_value != 0) {
            std::cout << "q " << q_value << std::endl;
          }
        }
        if (q_value > current_max_value) {
          current_best_actions = {};
          current_best_actions.push_back(possible_actions[i]);
          current_max_value = q_value;
        } else if (q_value == current_max_value) {
          current_best_actions.push_back(possible_actions[i]);
          current_max_value = q_value;
        }
      }
    }
    if (current_best_actions.size() == 0) {
      return possible_actions[0];
    }
    if (current_best_actions.size() == 1) {
      return current_best_actions[0];
    }
    return current_best_actions[(rand() % current_best_actions.size())];
  }
  /*****************************************************************************
   * GENERATE RANDOM marbleS
   * **************************************************************************/
  void generate_random_marbles(void) {
    if (random_marbles) {
      for (int x = 0; x < columns; x++) {
        for (int y = 0; y < rows; y++) {

          if (environment[x][y] != '#' && environment[x][y] != 's') {
            float marble_chance =
                (static_cast<float>(rand()) /
                 (static_cast<float>(static_cast<float>(RAND_MAX))));
            if (marble_chance < marble_map[x][y]) {
              environment[x][y] = 'c';
              marble_locations.push_back(get_room_index(x, y));
            } else {
              environment[x][y] = ' ';
            }
          }
        }
      }
    }
  }

  /*****************************************************************************
   * PRINT THE ENVIRONMENT
   * **************************************************************************/
  void print_environment() {
    for (int x = 0; x < columns; x++) {
      for (int y = 0; y < rows; y++) {
        if (environment[x][y] == '#' || environment[x][y] == 's' ||
            environment[x][y] == 'c') {
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
        int room_index = get_room_index(x, y);
        if (room_index != -1) {
          printf("%2.2i", room_index);
          // std::cout << room_index;
        } else {
          // std::cout << '#';
          printf("--");
        }
        std::cout << ' ';
      }
      std::cout << std::endl;
    }
  }
  /*****************************************************************************
   * PRINT MARBLE INDEXES
   * **************************************************************************/
  void print_marble_chances() {
    for (int x = 0; x < columns; x++) {
      for (int y = 0; y < rows; y++) {
        float marble_room_index = marble_map[x][y];
        if (marble_room_index != 0) {
          printf("%2.2f", marble_room_index);
        } else {
          printf("----");
        }
        std::cout << ' ';
      }
      std::cout << std::endl;
    }
  }

  /*****************************************************************************
   * PRINT BEST PATH LEARNED
   * **************************************************************************/
  void print_learned_path(int _time_to_live = 100) {
    q_learning::state Stest = {start_x, start_y, false, 0};
    while (!Stest.is_outside_environment && _time_to_live > 0) {
      std::cout << get_room_index(Stest.x, Stest.y) << " ";
      q_learning::action Atest = get_best_action(Stest);
      q_learning::state Stest_next = get_next_state(Stest, Atest);
      Stest = Stest_next;
      _time_to_live--;
    }
    std::cout << std::endl;
  }
  /*****************************************************************************
   * RETURN BEST PATH LEARNED
   * **************************************************************************/
  std::vector<int> return_learned_path(int _time_to_live = 100) {
    q_learning::state Stest = {start_x, start_y, false, 0};
    std::vector<int> pathing = {};
    while (!Stest.is_outside_environment && _time_to_live > 0) {
      q_learning::action Atest = get_best_action(Stest);
      q_learning::state Stest_next = get_next_state(Stest, Atest);
      pathing.push_back(get_room_index(Stest.x, Stest.y));
      Stest = Stest_next;
      _time_to_live--;
    }
    return pathing;
  }

  /*****************************************************************************
   * CHECK IF 2 VECTORS ARE COMPARABLE
   * **************************************************************************/
  static bool is_vector_comparable(std::vector<int> s1, std::vector<int> s2) {
    if (s1.size() != s2.size()) {
      return false;
    }
    for (size_t i = 0; i < s2.size(); i++) {
      if (s1[i] != s2[i]) {
        return false;
      }
    }
    return true;
  }

  /*****************************************************************************
   * CHECK IF CONTENT OF ONE VECTOR IS CONTAINED INSIDE ANOTHER
   * **************************************************************************/
  static bool is_vector_contained_inside(std::vector<int> _test_vector,
                                         std::vector<int> _refrence_vector,
                                         int _option = 0) {

    if (_option) {

      std::cout << "size: " << _test_vector.size() << std::endl;
      for (size_t i = 0; i < _test_vector.size(); i++) {
        std::cout << _test_vector[i] << " " << std::endl;
      }
      std::cout << std::endl;
    }
    size_t contains = 0;
    for (size_t i = 0; i < _refrence_vector.size(); i++) {
      for (size_t j = 0; j < _test_vector.size(); j++) {
        if (_refrence_vector[i] == _test_vector[j]) {
          contains++;
        }
      }
    }
    if (contains == _refrence_vector.size()) {
      if (_option) {
        std::cout << "contained inside is true!" << std::endl;
      }
      return true;
    }
    if (_option) {
      std::cout << "contained inside NOT true!" << std::endl;
    }
    return false;
  }
};
