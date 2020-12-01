#include "QLearning.h"
#include <cstddef>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <stdio.h>
#include <bitset>

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

  std::vector<std::vector<int>> char_map = {
      {'#', '#', '#', '#', '#', '#', '#', '#', '#'},
      {'#', '#', '#', '7', '#', '#', '#', '#', '#'},
      {'#', '5', '#', '2', '8', '#', '#', '#', '#'},
      {'#', '4', '3', ' ', '#', '#', '#', '#', '#'},
      {'#', '#', ' ', 's', '9', 'a', '#', '#', '#'},
      {'#', '#', '2', ' ', '#', 'b', 'c', 'd', '#'},
      {'#', '#', '#', '1', '#', 'e', '#', '#', '#'},
      {'#', '#', '#', '#', '#', 'f', '2', '#', '#'},
      {'#', '#', '#', '#', '#', '#', '#', '#', '#'}};

  float epsilon = 0.05; /*greedy value*/
  float alpha = 0.1; /*step size*/

  QLearning mylearning(char_map, epsilon);
  mylearning.print_environment();
  mylearning.print_rooms();

  std::cout << "PRINT ENVIROMENT" << std::endl;
  // Reset all state value estimates to 0:
  for (int y = 0; y < mylearning.rows; y++)
    for (int x = 0; x < mylearning.columns; x++)
      mylearning.V[y][x] = 0;

  int sweep = 0;
  float delta;

  size_t iterations = 10;//0000;
  int start_x = 4;
  int start_y = 4;
  // Start of the estimation loop
  std::cout << "MAIN LOOP" << std::endl;

  std::bitset<20> visited_list;


  //Loop for each episode
  for(size_t j = 0; j < iterations; j++){
      // initialise S
      QLearning::state S = {start_x,start_y,false};


      //Loop for each step of the episode
      while (!S.is_outside_environment) {
          //Choose A from S using policy derived from Q (e.g. epsilon-greedy)
          QLearning::action A = mylearning.get_next_action(S);

          //Take action A, observe R, S' (nextstate)
          QLearning::state S_next = mylearning.get_next_state(S,A); /*next state S'*/
          float R = mylearning.get_reward(S,A);


          //Update Q(S,A)
          QLearning::action a_max = mylearning.get_best_action(S_next);
          if(!S_next.is_outside_environment){
            mylearning.Q[S.x][S.y][A] = mylearning.Q[S.x][S.y][A] +
                    alpha * (R + mylearning.discount_rate * mylearning.Q[S_next.x][S_next.y][a_max]
                    - mylearning.Q[S.x][S.y][A]);
          } else{
              mylearning.Q[S.x][S.y][A] = mylearning.Q[S.x][S.y][A] +
                      alpha * (R + 0 - mylearning.Q[S.x][S.y][A]);
          }

          visited_list[mylearning.get_room_index(S.x,S.y)] = 1;
          //Update state - S <- S'
          S = S_next;

      } //until S is terminal

      mylearning.print_policy();
      visited_list.reset();
  }
}
