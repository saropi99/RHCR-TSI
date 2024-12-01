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
	timestep = 0;

	cout << "Randomly generating starts" << endl;
	initialize_start_locations();
	cout << "Randomly generating initial goals" << endl;
	initialize_goal_locations();
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
		used_starts.insert(loc);
	}
}

std::pair<int, int> MTSystem::generate_task() {
	auto pickup = this->G.random_location(this->rng);
	auto delivery = this->G.random_location(this->rng);
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
	for (int k = 0; k < num_of_drives; k++) {
		const auto number_of_goals = goal_locations[k].size();
		int curr = paths[k].back().location;
 
		int earliest_possible_finish = timestep;

		// recompute estimated finish times for existing goals
		for (size_t i = 0; i < goal_locations[k].size(); i++) {
			auto goal = goal_locations[k][i];
			auto dist = G.get_Manhattan_distance(curr, goal.first);
			earliest_possible_finish += dist;
			curr = goal.first;
		}

		// add new goals until until we have enough to fill the simulation window
		while (earliest_possible_finish <= timestep + simulation_window) {
			int pickup, delivery;
			std::tie(pickup, delivery) = generate_task();

			auto dist_to_pickup = G.get_Manhattan_distance(curr, pickup);
			earliest_possible_finish += dist_to_pickup;
			goal_locations[k].emplace_back(pickup, 0);

			auto dist_to_delivery = G.get_Manhattan_distance(pickup, delivery);
			earliest_possible_finish += dist_to_delivery;
			goal_locations[k].emplace_back(delivery, 0);

			curr = delivery;
		}

		const auto goals_added = goal_locations[k].size() - number_of_goals;
		assert(goals_added % 2 == 0);
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
		std::cout << "solving..." << std::endl;
        solve();

        // move drives
        auto new_finished_tasks = move();
        std::cout << new_finished_tasks.size() << " tasks have been finished" << std::endl;

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
	std::cout << "Total number of finished tasks: " << num_of_tasks << std::endl;
	save_results();
}
