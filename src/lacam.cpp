#include "lacam.h"
#include <lacam.hpp>
#include <algorithm>

LaCAM::LaCAM(BasicGraph& G, SingleAgentSolver& path_planner) : MAPFSolver(G, path_planner) {}

bool LaCAM::run(const vector<State>& starts,
                const vector< vector<pair<int, int> > >& goal_locations,
                int time_limit)
{
    clock_t start = std::clock();

    const auto N = starts.size();
    std::vector<int> start_indexes;
    for (const auto& start : starts)
    {
        start_indexes.push_back(start.location);
    }
    std::vector<std::vector<int>> goal_index_sequences;
    for (const auto& goal_sequence : goal_locations)
    {
        std::vector<int> goal_index_sequence;
        for (const auto& goal : goal_sequence)
        {
            goal_index_sequence.push_back(goal.first);
        }
        goal_index_sequences.push_back(goal_index_sequence);
    }

    lacam::Planner::FLG_STAR = false;
    lacam::Planner::FLG_SCATTER = false;

    auto ins = lacam::Instance(G.map_name + ".map", start_indexes, goal_index_sequences);
    assert(ins.is_valid(1));
    const auto verbosity = 10;
    const auto total_goals = ins.get_total_goals();
    const auto threshold = std::max(((int)total_goals * 9) / 10, 1);
    std::cout << "threshold: " << threshold << " (total_goals: " << total_goals << ")" << std::endl;
    auto deadline = lacam::Deadline(10 * 60 * 1000);  // 10min
    auto lacam_soln = lacam::solve(ins, threshold, verbosity, &deadline, 0);
    if (lacam::is_feasible_solution(ins, lacam_soln, threshold, verbosity)) {
        solution = std::vector<Path>(N);
        for (size_t t = 0; t < lacam_soln.size(); t++)
        {
            for (size_t i = 0; i < N; i++)
            {
                auto loc = lacam_soln[t][i];
                solution[i].push_back(State(loc->index));
            }
        }

        runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;
        solution_found = true;
        return true;
    }
    runtime = (std::clock() - start) * 1.0  / CLOCKS_PER_SEC;
    solution_cost = -1;
    solution_found = false;
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