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
      {'#', '#', '#', 'c', '#', '#', '#', '#', '#'},
      {'#', 'c', '#', ' ', ' ', '#', '#', '#', '#'},
      {'#', ' ', ' ', ' ', '#', '#', '#', '#', '#'},
      {'#', '#', ' ', 's', ' ', ' ', '#', '#', '#'},
      {'#', '#', ' ', ' ', '#', ' ', ' ', 'c', '#'},
      {'#', '#', '#', 'c', '#', ' ', '#', '#', '#'},
      {'#', '#', '#', '#', '#', ' ', 'c', '#', '#'},
      {'#', '#', '#', '#', '#', '#', '#', '#', '#'}};

  std::vector<std::vector<float>> marble_map = {
      {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f, 0.8f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      {0.0f, 0.8f, 0.0f, 0.1f, 0.02f, 0.0f, 0.0f, 0.0f, 0.0f},
      {0.0f, 0.2f, 0.05f, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.1f, 0.0f, 0.01f, 0.2f, 0.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.5f, 0.02f, 0.0f, 0.15f, 0.01f, 0.9f, 0.0f},
      {0.0f, 0.0f, 0.0f, 0.4f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.9f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};

  std::vector<std::vector<int>> test_map = {{'#', '#', '#', '#', '#', '#'},
                                            {'#', '#', ' ', '#', '#', '#'},
                                            {'#', ' ', 's', 'c', '#', '#'},
                                            {'#', '#', ' ', ' ', '#', '#'},
                                            {'#', '#', '#', '#', '#', '#'}};

  std::vector<std::vector<std::vector<int>>> maps;
  maps.push_back(test_map);
  maps.push_back(map1);
  float epsilon_value = 0.2; /*greedy value*/
  float alpha = 0.1;         /*step size*/

  QLearning q_learner(maps[1], marble_map, epsilon_value, RANDOM_MARBLES);
  std::cout << "PRINT ENVIRONMENT" << std::endl;
  q_learner.print_environment();
  q_learner.print_rooms();
  q_learner.print_marble_chances();

  size_t iterations = 200000;
  // Start of the estimation loop
  std::cout << "MAIN LOOP" << std::endl;

  /*****************************************************************************
   * LEARNING Loop for each episode
   * **************************************************************************/
  for (size_t j = 0; j < iterations; j++) {
    //    std::cout << "Learning Iteration " << j << "'s path: " << std::endl;
    // initialise S
    QLearning::state S = {q_learner.start_x, q_learner.start_y, false, 0};

    /***************************************************************************
     * EPISODE LOOP Loop for each step of the episode
     * ************************************************************************/
    while (!S.is_outside_environment) {
      //      std::cout << "room: " << q_learner.get_room_index(S.x, S.y) <<
      //      std::endl;
      // Choose A from S using policy derived from Q (e.g. epsilon-greedy)
      QLearning::action A = q_learner.get_next_action(S);

      // Take action A, observe R, S' (nextstate)
      QLearning::state S_next =
          q_learner.get_next_state(S, A); /*next state S'*/
      float R = q_learner.get_reward(S, A);

      // Update Q(S,A)
      QLearning::action a_max = q_learner.get_best_action(S_next);
      if (!S_next.is_outside_environment) {
        float updated_q_value =
            q_learner.get_q_value(S.x, S.y, A, S.visited_list) +
            alpha * (R +
                     q_learner.discount_rate *
                         q_learner.get_q_value(S_next.x, S_next.y, a_max,
                                               S_next.visited_list) -
                     q_learner.get_q_value(S.x, S.y, A, S.visited_list));
        q_learner.set_q_value(S.x, S.y, A, S.visited_list, updated_q_value);
        //        std::cout << "q_value: " << updated_q_value << std::endl;
      } else {
        float updated_q_value =
            q_learner.get_q_value(S.x, S.y, A, S.visited_list) +
            alpha *
                (R + 0 - q_learner.get_q_value(S.x, S.y, A, S.visited_list));
        q_learner.set_q_value(S.x, S.y, A, S.visited_list, updated_q_value);
        //        std::cout << "q_value: " << updated_q_value << std::endl;
      }
      S = S_next;
    } // until S is terminal

    //    std::cout << std::endl;

    q_learner.generate_random_marbles();
    /***************************************************************************
     * BEST PATH LEARNED DEBUG
     * ************************************************************************/
    if (j % ((iterations / 1) - 1) == 0) {
      std::cout << "Iteration " << j << "'s best path: ";
      QLearning::state Stest = {q_learner.start_x, q_learner.start_y, false, 0};
      int time_to_live = 100;
      while (!Stest.is_outside_environment && time_to_live > 0) {
        std::cout << q_learner.get_room_index(Stest.x, Stest.y) << " ";
        QLearning::action Atest = q_learner.get_best_action(Stest);
        QLearning::state Stest_next = q_learner.get_next_state(Stest, Atest);
        Stest = Stest_next;
        time_to_live--;
      }
      std::cout << std::endl;
    }
  }
}
