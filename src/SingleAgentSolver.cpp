#include "SingleAgentSolver.h"

const std::vector<double>& get_heuristic(BasicGraph& G, int location)
{
    if(G.heuristics.find(location) == G.heuristics.end())
    {
        // std::cout << "calculating heuristic for " << G.human_readable_loc(location) << "..." << std::flush;
        G.heuristics[location] = G.compute_heuristics(location);
        // std::cout << "done" << std::endl;
    }
    return G.heuristics[location];
}

double SingleAgentSolver::compute_h_value(BasicGraph& G, int curr, int goal_id,
                             const vector<pair<int, int> >& goal_location) const
{
    double h = get_heuristic(G, goal_location[goal_id].first)[curr];
    goal_id++;
    while (goal_id < (int) goal_location.size())
    {
        h += get_heuristic(G, goal_location[goal_id].first)[goal_location[goal_id - 1].first];
        goal_id++;
    }
    return h;
}
