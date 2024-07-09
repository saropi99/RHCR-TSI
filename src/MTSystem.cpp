#include "MTSystem.h"

MTSystem::MTSystem(const MTGrid& G, MAPFSolver& solver): BasicSystem(G, solver), G(G) {}

MTSystem::~MTSystem()
{
}

void MTSystem::simulate(int simulation_time)
{
    // initialize();
    // initialize_start_locations();
    // initialize_goal_locations();
    // update_goal_locations();
}
