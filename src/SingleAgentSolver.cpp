#include "SingleAgentSolver.h"


double SingleAgentSolver::compute_h_value(BasicGraph& G, int curr, int goal_id,
                             const vector<pair<int, int> >& goal_location) const
{
    assert(goal_id < goal_location.size());
    auto goal = goal_location[goal_id].first;
    if(!G.heuristics.contains(goal))
    {
        cout << "computing heuristic for " << goal << endl;
        G.heuristics[goal] = G.compute_heuristics(goal);
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
