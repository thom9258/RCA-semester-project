#include "QLearning.h"
#include <cstddef>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <stdio.h>

int main() {

  //  std::vector<std::vector<int>> char_map = {
  //      {'#', '#', '#', '#', '#', '#', '#', '#', '#'},
  //      {'#', '#', '#', '7', '#', '#', '#', '#', '#'},
  //      {'#', '5', '#', '6', '8', '#', '#', '#', '#'},
  //      {'#', '4', '3', ' ', '#', '#', '#', '#', '#'},
  //      {'#', '#', ' ', 's', '9', 'a', '#', '#', '#'},
  //      {'#', '#', '2', ' ', '#', 'b', 'c', 'd', '#'},
  //      {'#', '#', '#', '1', '#', 'e', '#', '#', '#'},
  //      {'#', '#', '#', '#', '#', 'f', 'g', '#', '#'},
  //      {'#', '#', '#', '#', '#', '#', '#', '#', '#'}};

  std::vector<std::vector<int>> map1 = {
      {'#', '#', '#', '#', '#', '#', '#', '#', '#'},
      {'#', '#', '#', ' ', '#', '#', '#', '#', '#'},
      {'#', ' ', '#', ' ', ' ', '#', '#', '#', '#'},
      {'#', ' ', ' ', ' ', '#', '#', '#', '#', '#'},
      {'#', '#', ' ', 's', ' ', ' ', '#', '#', '#'},
      {'#', '#', ' ', ' ', '#', ' ', ' ', ' ', '#'},
      {'#', '#', '#', ' ', '#', ' ', '#', '#', '#'},
      {'#', '#', '#', '#', '#', ' ', 'c', '#', '#'},
      {'#', '#', '#', '#', '#', '#', '#', '#', '#'}};

  std::vector<std::vector<int>> test_map = {{'#', '#', '#', '#', '#', '#'},
                                            {'#', '#', ' ', '#', '#', '#'},
                                            {'#', ' ', 's', 'c', '#', '#'},
                                            {'#', '#', ' ', ' ', '#', '#'},
                                            {'#', '#', '#', '#', '#', '#'}};

  std::vector<std::vector<std::vector<int>>> maps;
  maps.push_back(test_map);
  maps.push_back(map1);
  float epsilon_value = 0.5; /*greedy value*/
  float alpha = 1;           /*step size*/

  QLearning mylearning(maps[1], epsilon_value);
  std::cout << "PRINT ENVIROMENT" << std::endl;
  mylearning.print_environment();
  mylearning.print_rooms();

  size_t iterations = 100000;
  // Start of the estimation loop
  std::cout << "MAIN LOOP" << std::endl;

  /*****************************************************************************
   * LEARNING Loop for each episode
   * **************************************************************************/
  for (size_t j = 0; j < iterations; j++) {
    //    std::cout << "Learning Iteration " << j << "'s path: " << std::endl;
    // initialise S
    QLearning::state S = {mylearning.start_x, mylearning.start_y, false, 0};

    /***************************************************************************
     * EPISODE LOOP Loop for each step of the episode
     * ************************************************************************/
    while (!S.is_outside_environment) {
      //      std::cout << "room: " << mylearning.get_room_index(S.x, S.y) <<
      //      std::endl;
      // Choose A from S using policy derived from Q (e.g. epsilon-greedy)
      QLearning::action A = mylearning.get_next_action(S);

      // Take action A, observe R, S' (nextstate)
      QLearning::state S_next =
          mylearning.get_next_state(S, A); /*next state S'*/
      float R = mylearning.get_reward(S, A);

      // Update Q(S,A)
      QLearning::action a_max = mylearning.get_best_action(S_next);
      if (!S_next.is_outside_environment) {
        float updated_q_value =
            mylearning.get_q_value(S.x, S.y, A, S.visited_list) +
            alpha * (R +
                     mylearning.discount_rate *
                         mylearning.get_q_value(S_next.x, S_next.y, a_max,
                                                S_next.visited_list) -
                     mylearning.get_q_value(S.x, S.y, A, S.visited_list));
        mylearning.set_q_value(S.x, S.y, A, S.visited_list, updated_q_value);
        //        std::cout << "q_value: " << updated_q_value << std::endl;
      } else {
        float updated_q_value =
            mylearning.get_q_value(S.x, S.y, A, S.visited_list) +
            alpha *
                (R + 0 - mylearning.get_q_value(S.x, S.y, A, S.visited_list));
        mylearning.set_q_value(S.x, S.y, A, S.visited_list, updated_q_value);
        //        std::cout << "q_value: " << updated_q_value << std::endl;
      }
      S = S_next;
    } // until S is terminal

    //    std::cout << std::endl;

    /***************************************************************************
     * BEST PATH LEARNED DEBUG
     * ************************************************************************/
    if (j % ((iterations / 1) - 1) == 0) {
      std::cout << "Iteration " << j << "'s best path: ";
      QLearning::state Stest = {mylearning.start_x, mylearning.start_y, false,
                                0};
      int time_to_live = 100;
      while (!Stest.is_outside_environment && time_to_live > 0) {
        std::cout << mylearning.get_room_index(Stest.x, Stest.y) << " ";
        QLearning::action Atest = mylearning.get_best_action(Stest);
        QLearning::state Stest_next = mylearning.get_next_state(Stest, Atest);
        Stest = Stest_next;
        time_to_live--;
      }
      std::cout << std::endl;
    }
  }
}
