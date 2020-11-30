#include "QLearning.h"

// Dimensions of the environment
#define COLUMNS 4
#define ROWS 3

int main()
{
    QLearning mylearning;
    std::cout << "hello" << std::endl;
    std::cout << "Environment:" << std::endl;
    mylearning.PrintEnvironment();
    
    // Reset all state value estimates to 0:
    for (int y = 0; y < ROWS; y++)
        for (int x = 0; x < COLUMNS; x++)
            mylearning.V[y][x] = 0;
    
    
    int sweep   = 0;
    float delta;
    
    // Start of the estimation loop
    do {
        delta = 0;
        std::cout << "Current policy:" << std::endl;
        mylearning.PrintPolicy();
        
        // Perform a full sweep over the whole state space:
        for (int y = 0; y < ROWS; y++)
        {
            for (int x = 0; x < COLUMNS; x++)
            {
                QLearning::state s = { x, y };
                if (mylearning.environment[y][x] == ' ')
                {
                    float v      = mylearning.V[y][x];
                    QLearning::action a     = mylearning.GetNextAction(s);
                    float reward = mylearning.GetReward(s, a);
                    QLearning::state next   = mylearning.GetNextState(s, a);
                    if (!next.is_outside_environment)
                        mylearning.V[y][x] = reward + mylearning.discount_rate * mylearning.V[next.y][next.x];
                    
                    delta = std::max(delta, (float) fabs(v - mylearning.V[y][x]));
                }
            }
        }
        
        std::cout << "Sweep #" << ++sweep << " delta: " << delta << std::endl;
        mylearning.PrintStateValues();
    } while (delta > mylearning.theta); // Check if our currect estimate is accurate enough.
}
