#include "QLearning.h"
#include <cstddef>
#include <iostream>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <stdexcept>
#include <stdio.h>
int main() {

  std::vector<cv::Mat> maps = {};
  maps.push_back(cv::imread("./maps/floor_plan_small.png"));
  maps.push_back(cv::imread("./maps/floor_plan_large.png"));

  QLearning mylearning(maps[0]);
  std::cout << "hello" << std::endl;
  std::cout << "Environment:" << std::endl;
  mylearning.PrintEnvironment();

  // Reset all state value estimates to 0:
  for (int y = 0; y < mylearning.rows; y++)
    for (int x = 0; x < mylearning.columns; x++)
      mylearning.V[y][x] = 0;

  int sweep = 0;
  float delta;

  // Start of the estimation loop
  do {
    delta = 0;
    std::cout << "Current policy:" << std::endl;
    mylearning.PrintPolicy();

    // Perform a full sweep over the whole state space:
    for (int y = 0; y < mylearning.rows; y++) {
      for (int x = 0; x < mylearning.columns; x++) {
        QLearning::state s = {x, y};
        if (mylearning.environment[y][x] == ' ') {
          float v = mylearning.V[y][x];
          QLearning::action a = mylearning.GetNextAction(s);
          float reward = mylearning.GetReward(s, a);
          QLearning::state next = mylearning.GetNextState(s, a);
          if (!next.is_outside_environment)
            mylearning.V[y][x] = reward + mylearning.discount_rate *
                                              mylearning.V[next.y][next.x];

          delta = std::max(delta, (float)fabs(v - mylearning.V[y][x]));
        }
      }
    }

    std::cout << "Sweep #" << ++sweep << " delta: " << delta << std::endl;
    mylearning.PrintStateValues();
  } while (
      delta >
      mylearning.theta); // Check if our currect estimate is accurate enough.
}
