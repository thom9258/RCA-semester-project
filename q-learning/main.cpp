#include "q_learning.h"
#include <cstddef>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <stdio.h>

int main() {
  std::vector<std::vector<int>> test_map = {{'#', '#', '#', '#', '#', '#'},
                                            {'#', '#', ' ', '#', '#', '#'},
                                            {'#', ' ', 's', 'c', '#', '#'},
                                            {'#', '#', ' ', ' ', '#', '#'},
                                            {'#', '#', '#', '#', '#', '#'}};

  std::vector<std::vector<int>> room_map = {
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

  std::vector<std::vector<std::vector<int>>> maps;
  maps.push_back(test_map);
  maps.push_back(room_map);

  /*TEST CONTROL*/
  bool test_1 = false;
  bool test_2 = false;
  bool test_3 = true;
  bool test_4 = false;
  bool test_5 = false;

  /*****************************************************************************
   * TEST 1:
   * fixed marble, test for deterministic path, alpha = 1
   * **************************************************************************/
  if (test_1) {
    std::vector<std::vector<int>> test_map = {
        {'#', '#', '#', '#', '#', '#', '#', '#', '#'},
        {'#', '#', '#', 'c', '#', '#', '#', '#', '#'},
        {'#', 'c', '#', ' ', ' ', '#', '#', '#', '#'},
        {'#', ' ', ' ', ' ', '#', '#', '#', '#', '#'},
        {'#', '#', ' ', 's', ' ', ' ', '#', '#', '#'},
        {'#', '#', ' ', ' ', '#', ' ', ' ', 'c', '#'},
        {'#', '#', '#', 'c', '#', ' ', '#', '#', '#'},
        {'#', '#', '#', '#', '#', ' ', ' ', '#', '#'},
        {'#', '#', '#', '#', '#', '#', '#', '#', '#'}};

    int iterations = 2000;
    float epsilon_value = 0.2; /*greedy value*/
    float alpha = 1;           /*step size*/
    int MAX_TEST_ITERATIONS = 10;
    int time_to_live = 25;

    for (int i = 0; i < MAX_TEST_ITERATIONS; i++) {
      q_learning q_learner(test_map, marble_map, epsilon_value);
      q_learner.print_environment(1);
      q_learner.print_rooms();
      //      q_learner.print_marble_chances();
      /*
       * LEARNING LOOP FOR EACH EPISODE
       * */
      for (int j = 0; j < iterations; j++) {
        // initialise S
        q_learning::state S = {q_learner.start_x, q_learner.start_y, false, 0};

        /*
         * EPISODE LOOP FOR EACH STEP OF THE EPISODE
         * */
        while (!S.is_outside_environment) {

          // Choose A from S using policy derived from Q (e.g. epsilon-greedy)
          q_learning::action A = q_learner.get_next_action(S);

          // Take action A, observe R, S' (nextstate)
          q_learning::state S_next =
              q_learner.get_next_state(S, A); /*next state S'*/
          float R = q_learner.get_reward(S, A);

          // Update Q(S,A)
          q_learning::action a_max = q_learner.get_best_action(S_next);
          if (!S_next.is_outside_environment) {
            float updated_q_value =
                q_learner.get_q_value(S.x, S.y, A, S.visited_list) +
                alpha * (R +
                         q_learner.discount_rate *
                             q_learner.get_q_value(S_next.x, S_next.y, a_max,
                                                   S_next.visited_list) -
                         q_learner.get_q_value(S.x, S.y, A, S.visited_list));
            q_learner.set_q_value(S.x, S.y, A, S.visited_list, updated_q_value);
          } else {
            float updated_q_value =
                q_learner.get_q_value(S.x, S.y, A, S.visited_list) +
                alpha * (R + 0 -
                         q_learner.get_q_value(S.x, S.y, A, S.visited_list));
            q_learner.set_q_value(S.x, S.y, A, S.visited_list, updated_q_value);
          }
          S = S_next;
        } /*until S is terminal*/
      }

      /*
       * BEST PATH LEARNED DEBUG
       * */
      std::cout << i << "'th best path learned: " << std::endl;
      q_learner.print_learned_path(time_to_live);
    }
  }
  /*****************************************************************************
   * TEST 2:
   * fixed marble, test for deterministic path, alpha = 1
   * **************************************************************************/
  if (test_2) {
    std::vector<std::vector<int>> test_map = {
        {'#', '#', '#', '#', '#', '#', '#', '#', '#'},
        {'#', '#', '#', ' ', '#', '#', '#', '#', '#'},
        {'#', ' ', '#', ' ', ' ', '#', '#', '#', '#'},
        {'#', ' ', ' ', ' ', '#', '#', '#', '#', '#'},
        {'#', '#', ' ', 's', ' ', ' ', '#', '#', '#'},
        {'#', '#', ' ', ' ', '#', ' ', ' ', 'c', '#'},
        {'#', '#', '#', ' ', '#', ' ', '#', '#', '#'},
        {'#', '#', '#', '#', '#', ' ', ' ', '#', '#'},
        {'#', '#', '#', '#', '#', '#', '#', '#', '#'}};

    /*
     * Note:
     * expected pathing is determined based on this indexing:
     *      -- -- -- -- -- -- -- -- --
     *      -- -- -- 00 -- -- -- -- --
     *      -- 01 -- 02 03 -- -- -- --
     *      -- 04 05 06 -- -- -- -- --
     *      -- -- 07 08 09 10 -- -- --
     *      -- -- 11 12 -- 13 14 15 --
     *      -- -- -- 16 -- 17 -- -- --
     *      -- -- -- -- -- 18 19 -- --
     *      -- -- -- -- -- -- -- -- --
     * */
    std::vector<int> expected_pathing = {8, 9, 10, 13, 14, 15};
    int iterations = 2000;
    float epsilon_value = 0.01; /*greedy value*/
    float alpha = 1;            /*step size*/
    int time_to_live = 25;
    std::ofstream output;
    output.open("best_epsilon_value_static_marble.csv");

    while (epsilon_value < 1.0f) {
      q_learning q_learner(test_map, marble_map, epsilon_value);

      /*
       * LEARNING LOOP FOR EACH EPISODE
       * */
      for (int j = 0; j < iterations; j++) {
        // initialise S
        q_learning::state S = {q_learner.start_x, q_learner.start_y, false, 0};

        /*
         * EPISODE LOOP FOR EACH STEP OF THE EPISODE
         * */
        while (!S.is_outside_environment) {

          // Choose A from S using policy derived from Q (e.g. epsilon-greedy)
          q_learning::action A = q_learner.get_next_action(S);

          // Take action A, observe R, S' (nextstate)
          q_learning::state S_next =
              q_learner.get_next_state(S, A); /*next state S'*/
          float R = q_learner.get_reward(S, A);

          // Update Q(S,A)
          q_learning::action a_max = q_learner.get_best_action(S_next);
          if (!S_next.is_outside_environment) {
            float updated_q_value =
                q_learner.get_q_value(S.x, S.y, A, S.visited_list) +
                alpha * (R +
                         q_learner.discount_rate *
                             q_learner.get_q_value(S_next.x, S_next.y, a_max,
                                                   S_next.visited_list) -
                         q_learner.get_q_value(S.x, S.y, A, S.visited_list));
            q_learner.set_q_value(S.x, S.y, A, S.visited_list, updated_q_value);
          } else {
            float updated_q_value =
                q_learner.get_q_value(S.x, S.y, A, S.visited_list) +
                alpha * (R + 0 -
                         q_learner.get_q_value(S.x, S.y, A, S.visited_list));
            q_learner.set_q_value(S.x, S.y, A, S.visited_list, updated_q_value);
          }
          S = S_next;
        } /*until S is terminal*/

        /*
         * Check if path is correct
         * */
        std::vector<int> learned_path =
            q_learner.return_learned_path(time_to_live);
        while (learned_path.size() > expected_pathing.size()) {
          learned_path.pop_back();
        }
        if (q_learning::is_vector_comparable(learned_path, expected_pathing)) {
          std::cout << epsilon_value << "," << j << std::endl;
          output << epsilon_value << "," << j << std::endl;
          epsilon_value += 0.01;
          break;
        }
      }
    }
    output.close();
  }
  /*****************************************************************************
   * TEST 3:
   * random marble, find epsilon and alpha
   * **************************************************************************/
  if (test_3) {
    std::vector<std::vector<int>> room_map = {
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

    std::vector<int> all_rooms = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
                                  10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    int iterations = 10000;
    float epsilon_value = 1.0f; /*greedy value*/
    float alpha = 1.0f;         /*step size*/
    int time_to_live = 1000;
    std::ofstream output;
    output.open("epsilon_alpha_iterations_random_marbles.csv");

    for (; epsilon_value <= 100.0f; epsilon_value = epsilon_value + 1.0f) {
      for (; alpha <= 100.0f; alpha = alpha + 1.0f) {

        q_learning q_learner(room_map, marble_map, epsilon_value / 100,
                             RANDOM_MARBLES);
        //        q_learner.generate_random_marbles();
        //        std::cout << "NEW ENVIROMENT ---------------------" <<
        //        std::endl; q_learner.print_environment();
        /*
         * LEARNING LOOP FOR EACH EPISODE
         * */
        for (int j = 0; j < iterations; j++) {
          // initialise S
          q_learning::state S = {q_learner.start_x, q_learner.start_y, false,
                                 0};

          /*
           * EPISODE LOOP FOR EACH STEP OF THE EPISODE
           * */
          while (!S.is_outside_environment) {

            // Choose A from S using policy derived from Q (e.g. epsilon-greedy)
            q_learning::action A = q_learner.get_next_action(S);

            // Take action A, observe R, S' (nextstate)
            q_learning::state S_next =
                q_learner.get_next_state(S, A); /*next state S'*/
            float R = q_learner.get_reward(S, A);

            // Update Q(S,A)
            q_learning::action a_max = q_learner.get_best_action(S_next);
            if (!S_next.is_outside_environment) {
              float updated_q_value =
                  q_learner.get_q_value(S.x, S.y, A, S.visited_list) +
                  (alpha / 100) *
                      (R +
                       q_learner.discount_rate *
                           q_learner.get_q_value(S_next.x, S_next.y, a_max,
                                                 S_next.visited_list) -
                       q_learner.get_q_value(S.x, S.y, A, S.visited_list));
              q_learner.set_q_value(S.x, S.y, A, S.visited_list,
                                    updated_q_value);
            } else {
              float updated_q_value =
                  q_learner.get_q_value(S.x, S.y, A, S.visited_list) +
                  (alpha / 100) *
                      (R + 0 -
                       q_learner.get_q_value(S.x, S.y, A, S.visited_list));
              q_learner.set_q_value(S.x, S.y, A, S.visited_list,
                                    updated_q_value);
            }
            S = S_next;
          } /*until S is terminal*/

          /*
           * Check if path is correct
           * */
          std::vector<int> learned_path =
              q_learner.return_learned_path(time_to_live);

          if (j == iterations - 1) {
            std::cout << "DATA: " << epsilon_value / 100 << "," << alpha / 100
                      << "," << -1 << std::endl;
            //            output << epsilon_value / 100 << "," << alpha / 100 <<
            //            "," << -1
            //                   << std::endl;
            break;
          } else if (q_learning::is_vector_contained_inside(learned_path,
                                                            all_rooms, 1)) {
            std::cout << "DATA: " << epsilon_value / 100 << "," << alpha / 100
                      << "," << j << std::endl;
            output << epsilon_value / 100 << "," << alpha / 100 << "," << j
                   << std::endl;
            break;
          }
          q_learner.generate_random_marbles();
        }
      } /*For alpha end*/
      alpha = 1.0f;
    } /*For epsilon end*/
    std::cout << "DONE" << std::endl;
    output.close();
  }
  /*****************************************************************************
   * TEST 4:
   * fixed marble, test for deterministic path, alpha = 1
   * **************************************************************************/
  if (test_4) {
  }
  /*****************************************************************************
   * TEST 5:
   * fixed marble, test for deterministic path, alpha = 1
   * **************************************************************************/
  if (test_5) {
  }
}
