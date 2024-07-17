#pragma once
#include "BasicSystem.h"
#include "MTGrid.h"
#include <random>

class MTSystem :
	public BasicSystem
{
public:
	MTSystem(MTGrid& G, MAPFSolver& solver);
	~MTSystem();

	void simulate(int simulation_time);

private:
	MTGrid& G;
	std::mt19937 rng;

	void initialize();
	void initialize_start_locations();
	void initialize_goal_locations();
	void update_goal_locations();
	std::pair<int, int> generate_task();
};

