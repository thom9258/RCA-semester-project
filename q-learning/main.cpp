#include "QLearning.h"
#include <cstddef>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <stdio.h>
int main() {

  std::vector<std::vector<int>> char_map = {
      {'#', '#', '#', '#', '#', '#', '#', '#', '#'},
      {'#', '#', '#', '7', '#', '#', '#', '#', '#'},
      {'#', '5', '#', '6', '8', '#', '#', '#', '#'},
      {'#', '4', '3', ' ', '#', '#', '#', '#', '#'},
      {'#', '#', ' ', 's', '9', 'a', '#', '#', '#'},
      {'#', '#', '2', ' ', '#', 'b', 'c', 'd', '#'},
      {'#', '#', '#', '1', '#', 'e', '#', '#', '#'},
      {'#', '#', '#', '#', '#', 'f', 'g', '#', '#'},
      {'#', '#', '#', '#', '#', '#', '#', '#', '#'}};

  QLearning mylearning(char_map);
  mylearning.print_environment();

  std::cout << "PRINT ENVIROMENT" << std::endl;
  // Reset all state value estimates to 0:
  for (int y = 0; y < mylearning.rows; y++)
    for (int x = 0; x < mylearning.columns; x++)
      mylearning.V[y][x] = 0;

  int sweep = 0;
  float delta;

  // Start of the estimation loop
  std::cout << "MAIN LOOP" << std::endl;
  do {
    delta = 0;
    std::cout << "Current policy:" << std::endl;
    mylearning.print_policy();

    // Perform a full sweep over the whole state space:
    for (int y = 0; y < mylearning.rows; y++) {
      for (int x = 0; x < mylearning.columns; x++) {
        QLearning::state s = {x, y};
        if (mylearning.environment[y][x] != '#') {
          float v = mylearning.V[y][x];
          QLearning::action a = mylearning.get_next_action(s);
          float reward = mylearning.get_reward(s, a);
          QLearning::state next = mylearning.get_next_state(s, a);
          if (!next.is_outside_environment)
            mylearning.V[y][x] = reward + mylearning.discount_rate *
                                              mylearning.V[next.y][next.x];

          delta = std::max(delta, (float)fabs(v - mylearning.V[y][x]));
        }
      }
    }

    std::cout << "Sweep #" << ++sweep << " delta: " << delta << std::endl;
    mylearning.print_state_values();
  } while (
      delta >
      mylearning.theta); // Check if our currect estimate is accurate enough.
}
