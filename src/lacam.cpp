#include "lacam.h"

LaCAM::LaCAM(BasicGraph& G, SingleAgentSolver& path_planner) : MAPFSolver(G, path_planner) {}

bool LaCAM::run(const vector<State>& starts,
                const vector< vector<pair<int, int> > >& goal_locations,
                int time_limit)
{
    return false;
}

void LaCAM::save_results(const std::string &fileName, const std::string &instanceName) const
{
    // std::ofstream stats;
    // stats.open(fileName, std::ios::app);
    // stats << runtime << "," <<
    //          solution_cost << "," << min_sum_of_costs << "," <<
    //          avg_path_length << "," << "0" << "," <<
    //          instanceName << std::endl;
    // stats.close();
}

void LaCAM::clear()
{
	runtime = 0;
	solution_found = false;
	solution_cost = -2;
	avg_path_length = -1;
	solution.clear();
	initial_constraints.clear();
	initial_rt.clear();
}