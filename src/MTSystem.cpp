#include "MTSystem.h"

MTSystem::MTSystem(const MTGrid& G, MAPFSolver& solver): BasicSystem(G, solver), G(G) {}

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
	for (int k = 0; k < num_of_drives; k++)
	{
        int orientation = consider_rotation ? rand() % 4 : -1;
        starts[k] = State(0, 0, orientation);
		paths[k].emplace_back(starts[k]);
		finished_tasks[k].emplace_back(starts[k].location, 0);
	}
}


void MTSystem::initialize_goal_locations()
{
	if (hold_endpoints || useDummyPaths)
		return;
	// Choose random goal locations
	// Goal locations are not necessarily unique
	for (int k = 0; k < num_of_drives; k++)
	{
		int goal = G.endpoints[rand() % (int)G.endpoints.size()];
		goal_locations[k].emplace_back(goal, 0);
	}
}

void KivaSystem::update_goal_locations()
{
}

void MTSystem::simulate(int simulation_time)
{
    std::cout << "*** Simulating " << seed << " ***" << std::endl;
    this->simulation_time = simulation_time;
    initialize();

    for(; timestep < simulation_time; timestep += simulation_window)
    {
        std::cout << "Timestep " << timestep << std::endl;

        update_start_locations();
        update_goal_locations();
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
            if (hold_endpoints)
                held_endpoints.erase(loc);
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
