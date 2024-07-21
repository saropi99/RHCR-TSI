#include "MTSystem.h"

MTSystem::MTSystem(MTGrid& G, MAPFSolver& solver): BasicSystem(G, solver), G(G) {}

MTSystem::~MTSystem()
{
}

void MTSystem::initialize()
{
	initialize_solvers();

	starts.resize(num_of_drives);
	goal_locations.resize(num_of_drives);
	paths.resize(num_of_drives);
	finished_tasks.resize(num_of_drives);
	bool succ = load_records(); // continue simulating from the records
	if (!succ)
	{
		timestep = 0;
		succ = load_locations();
		if (!succ)
		{
			cout << "Randomly generating initial locations" << endl;
			initialize_start_locations();
			initialize_goal_locations();
		}
	}
}

void MTSystem::initialize_start_locations()
{
	// Choose random start locations
	// Any non-obstacle locations can be start locations
	// Start locations should be unique
	std::unordered_set<int> used_starts;
	for (int k = 0; k < num_of_drives; k++)
	{
		auto loc = this->G.random_location(this->rng);
		while(used_starts.count(loc)){
			loc = this->G.random_location(this->rng);
		}
        int orientation = consider_rotation ? rand() % 4 : -1;
        starts[k] = State(loc, 0, orientation);
		paths[k].emplace_back(starts[k]);
		finished_tasks[k].emplace_back(starts[k].location, 0);
	}
}

std::pair<int, int> MTSystem::generate_task() {
	auto pickup = this->G.random_location(this->rng);
	auto delivery = this->G.random_location(this->rng);
	std::cout << "Generated task: " << G.human_readable_loc(pickup) << " -> " << G.human_readable_loc(delivery) << std::endl;
	return make_pair(pickup, delivery);
}

void MTSystem::initialize_goal_locations()
{
	// Choose random goal locations
	// Goal locations are not necessarily unique
	for (int k = 0; k < num_of_drives; k++)
	{
		auto task = generate_task();
		goal_locations[k].emplace_back(task.first, 0);
		goal_locations[k].emplace_back(task.second, 0);
	}
}

void MTSystem::update_goal_locations()
{
	for (int k = 0; k < num_of_drives; k++)
	{
		std::pair<int, int> curr(paths[k][timestep].location, timestep);
		std::pair<int, int> goal = goal_locations[k].empty() ? curr : goal_locations[k].back();

		int min_timesteps = G.get_Manhattan_distance(curr.first, goal.first);
		min_timesteps = max(min_timesteps, goal.second);
		while (min_timesteps <= simulation_window) {
			auto next = generate_task();
			goal_locations[k].emplace_back(next.first, 0);
			goal_locations[k].emplace_back(next.second, 0);
			min_timesteps += G.get_Manhattan_distance(goal.first, next.first);
			min_timesteps += G.get_Manhattan_distance(next.first, next.second);
			goal = make_pair(next.second, 0);
		}
	}
}

void MTSystem::simulate(int simulation_time)
{
    std::cout << "*** Simulating " << this->seed << " ***" << std::endl;
	this->rng = std::mt19937(this->seed);
    this->simulation_time = simulation_time;
    initialize();

    for(; timestep < simulation_time; timestep += simulation_window)
    {
        std::cout << "Timestep " << timestep << std::endl;

        update_start_locations();
        update_goal_locations();
		assert(starts.size() == goal_locations.size());
		// for(size_t i = 0; i < starts.size(); i++) {
		// 	std::cout << "Start: " << G.human_readable_loc(starts[i].location) << " @ " << starts[i].timestep << std::endl;
		// 	std::cout << "Goals: ";
		// 	for(auto& goal : goal_locations[i]) {
		// 		std::cout << G.human_readable_loc(goal.first) << " @ " << goal.second << ", ";
		// 	}
		// 	std::cout << std::endl;
		// }
		std::cout << "solving..." << std::endl;
        solve();

        // move drives
        auto new_finished_tasks = move();
        std::cout << new_finished_tasks.size() << " tasks have been finished:" << std::endl;
		for (auto task : new_finished_tasks)
		{
			int curr;
			std::tie(std::ignore, curr, std::ignore) = task;
			std::cout << G.human_readable_loc(curr) << std::endl;
		}

        // update tasks 
        for(auto task : new_finished_tasks)
        {
            int id, loc, t;
            std::tie(id, loc, t) = task;
			finished_tasks[id].emplace_back(loc, t);
            num_of_tasks++;
        }

		if (congested())
		{
			cout << "***** Too many traffic jams ***" << endl;
			break;
		}
    }

    update_start_locations();
	std::cout << std::endl << "Done!" << std::endl;
	save_results();
}
