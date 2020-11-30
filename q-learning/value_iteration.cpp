#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <math.h>

/**
 * Basic implementation of value iteration (see p. 83 in Sutton 
 * and Barto (2018) "Reinformcement Learning, an Introduction") for a deterministic 
 * 2D lattice environment.
 *
 * Written 2018 by Anders Lyhne Christensen.
 **/ 

// Dimensions of the environment
#define COLUMNS 4
#define ROWS 3
 
// Environment -- spaces: agent can move, "+": reward, "-": punishment.
char environment[ROWS][COLUMNS] = { { ' ', ' ', ' ', '+' },
                                    { ' ', '#', ' ', '-' },
						            { ' ', ' ', ' ', ' ' } };
						 
// Current estimate of state values under the current policy:
float V[ROWS][COLUMNS];						 
						 
// State is given by (x, y) in the environment. Must be inside the environment to be valid						 
struct state 
{
	int x;
	int y;
	bool is_outside_environment;
};

// A convenient definition of the terminal state
const state TERMINAL_STATE = { -1, -1, true };

// Discount rate:
float discount_rate = 0.9;

// Theta: the thredhold for determining the accuracy of the estimation
float theta = 0.01;

// Actions:
enum action { UP, DOWN, LEFT, RIGHT };

// Get the next state given a current state s and an action a:
state GetNextState(state s, action a) 
{
	if (environment[s.y][s.x] != ' ')
		return TERMINAL_STATE;

	switch (a) {
	case UP:     s.y -= 1; break;
	case DOWN:   s.y += 1; break;
	case LEFT:   s.x -= 1; break;
	case RIGHT:  s.x += 1; break;
	}

	if (s.x < 0 || s.y < 0 || s.x >= COLUMNS || s.y >= ROWS)
		return TERMINAL_STATE;
	
	s.is_outside_environment = false;
	return s;
}

// Ger the reward given a state and an action:
float GetReward(state s, action a) 
{
	state next = GetNextState(s, a);
	if (next.is_outside_environment)
	{
		return 0;
	} else {
		if (environment[next.y][next.x] == '+')
			return 1.0;
		
		if (environment[next.y][next.x] == '-')
			return -1.0;
		
		return 0;
	}
}

// Get the best action computed according to the current state-value estimate:
action GetNextAction(state s)
{
	std::vector<action> possible_actions = { UP, DOWN, LEFT, RIGHT };
	
	float current_max_value = std::numeric_limits<float>::min();
	action best_action      = possible_actions[0]; // Make sure that we have a default action (not really necessary)
	
	for (const auto a : possible_actions)
	{
		state next   = GetNextState(s, a);
		float reward = GetReward(s, a);
		if (!next.is_outside_environment) 
		{
			if (V[next.y][next.x] + reward > current_max_value)
			{
				best_action = a;
				current_max_value = V[next.y][next.x] + reward;
				
			}
		}
	}
	return best_action;
}

// Print the environment with border around:
void PrintEnvironment()
{
	for (int y = -1; y <= ROWS; y++)
	{
		for (int x = -1; x <= COLUMNS; x++)
			if (y < 0 || y >= ROWS || x < 0 || x >= COLUMNS)
				std::cout << "#";
			else	
				std::cout << environment[y][x];
		
		std::cout << std::endl;
	}
}

// Print the current estimate of state values:
void PrintStateValues() 
{
	for (int y = 0; y < ROWS; y++) 
	{
		for (int x = 0; x < COLUMNS; x++)
			printf(" %5.2f ", V[y][x]);
			
		printf("\n");
	}
}

// Print the current estimate of state values:
void PrintPolicy() 
{
	for (int y = 0; y < ROWS; y++) 
	{
		for (int x = 0; x < COLUMNS; x++)
		{
			if (environment[y][x] == ' ')
			{
				action a = GetNextAction((state) {x, y});
		
				switch(a) {
					case UP:    printf("  UP   "); break;
					case DOWN:  printf(" DOWN  "); break;
					case LEFT:  printf(" LEFT  "); break;
					case RIGHT: printf(" RIGHT "); break;
					default: 
						printf(" UNKNOWN ACTION! ");
				}
			} else {
				printf("   %c   ", environment[y][x]);
			}
		}	
		
		printf("\n");
	}
}


int main(int argc, char** argv) 
{	
	std::cout << "Environment:" << std::endl;
	PrintEnvironment();
	
	// Reset all state value estimates to 0:
	for (int y = 0; y < ROWS; y++)
		for (int x = 0; x < COLUMNS; x++)
			V[y][x] = 0;
	
	
	int sweep   = 0; 
	float delta;
	
	// Start of the estimation loop
	do {
		delta = 0;
		std::cout << "Current policy:" << std::endl;
		PrintPolicy();
		
        // Perform a full sweep over the whole state space: 	
		for (int y = 0; y < ROWS; y++)
		{
			for (int x = 0; x < COLUMNS; x++)
			{
				state s = { x, y };
				if (environment[y][x] == ' ') 
				{
					float v      = V[y][x];
					action a     = GetNextAction(s);
					float reward = GetReward(s, a);
					state next   = GetNextState(s, a);
					if (!next.is_outside_environment)
						V[y][x] = reward + discount_rate * V[next.y][next.x];
					
					delta = std::max(delta, (float) fabs(v - V[y][x]));
				}
			}
	    }
		
		std::cout << "Sweep #" << ++sweep << " delta: " << delta << std::endl;
		PrintStateValues();
	} while (delta > theta); // Check if our currect estimate is accurate enough.
};
