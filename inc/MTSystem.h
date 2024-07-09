#pragma once
#include "BasicSystem.h"
#include "MTGrid.h"

class MTSystem :
	public BasicSystem
{
public:
	MTSystem(const MTGrid& G, MAPFSolver& solver);
	~MTSystem();

	void simulate(int simulation_time);


private:
	const MTGrid& G;

	void initialize();
	void initialize_start_locations();
	void initialize_goal_locations();
	void update_goal_locations();
};

