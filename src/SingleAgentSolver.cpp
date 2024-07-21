#include "SingleAgentSolver.h"


double SingleAgentSolver::compute_h_value(BasicGraph& G, int curr, int goal_id,
                             const vector<pair<int, int> >& goal_location) const
{
    assert(goal_id < goal_location.size());
    auto goal = goal_location[goal_id].first;
    if(!G.heuristics.contains(goal))
    {
        std::cout << "calculating heuristic for " << G.human_readable_loc(goal) << "..." << std::flush;
        G.heuristics[goal] = G.compute_heuristics(goal);
        std::cout << "done" << std::endl;
    }
    double h = G.heuristics.at(goal)[curr];
    goal_id++;
    while (goal_id < (int) goal_location.size())
    {
        h += G.heuristics.at(goal)[goal_location[goal_id - 1].first];
        goal_id++;
    }
    return h;
}
