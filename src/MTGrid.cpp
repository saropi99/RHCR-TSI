#include "MTGrid.h"
#include <boost/tokenizer.hpp>

bool MTGrid::load_map(std::string fname)
{
    std::ifstream myfile(fname.c_str());
    if (!myfile.is_open())
    {
        std::cout << "Unable to open file " << fname << std::endl;
        return false;
    }
    std::cout << "*** Loading map ***" << std::endl;
    clock_t t = std::clock();
    this->map_name = fname.substr(0, fname.rfind('.')); // name without extension

    std::string line;
    getline(myfile, line); // skip "type octile"
    getline(myfile, line); // height XXX
    boost::char_separator<char> sep(" ");
    auto tok = boost::tokenizer(line, sep);
    auto beg = tok.begin();
    beg++;
    this->rows = atoi(beg->c_str());

    getline(myfile, line); // width XXX
    tok = boost::tokenizer(line, sep);
    beg = tok.begin();
    beg++;
    this->cols = atoi(beg->c_str());

    this->move[0] = 1;
    this->move[1] = -cols;
    this->move[2] = -1;
    this->move[3] = cols;

    types.resize(rows * cols);
    weights.resize(rows * cols);
    getline(myfile, line); // skip "map"
    for (int i = 0; i < this->rows; i++)
    {
        getline(myfile, line);
        for (int j = 0; j < this->cols; j++)
        {
            auto id = cols * i + j;
            weights[id].resize(5, WEIGHT_MAX);
            if (line[j] == '@') // obstacle
            {
                types[id] = "Obstacle";
            }
            else
            {
                types[id] = "Travel";
                weights[id][4] = 1;
            }
        }
    }
    for (int id = 0; id < cols * rows; id++)
    {
        if (types[id] == "Obstacle")
            continue;
        for (int dir = 0; dir < 4; dir++)
        {
            if (0 <= id + move[dir] && id + move[dir] < cols * rows && get_Manhattan_distance(id, id + move[dir]) <= 1 && types[id + move[dir]] != "Obstacle")
                weights[id][dir] = 1;
            else
                weights[id][dir] = WEIGHT_MAX;
        }
    }

    myfile.close();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Map size: " << rows << "x" << cols << std::endl;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}

void MTGrid::preprocessing(bool consider_rotation)
{
    std::cout << "*** Preprocessing map ***" << std::endl;
    clock_t t = std::clock();
	this->consider_rotation = consider_rotation;
	std::string fname;
	if (consider_rotation)
		fname = map_name + "_rotation_heuristics_table.txt";
	else
		fname = map_name + "_heuristics_table.txt";
	std::ifstream myfile(fname.c_str());
	bool succ = false;
	if (myfile.is_open())
	{
		succ = load_heuristics_table(myfile);
		myfile.close();
	}
	if (!succ)
	{
        for (int id = 0; id < rows*cols; id++) {
            heuristics[id] = compute_heuristics(id);
        }
		save_heuristics_table(fname);
	}

	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
}

int MTGrid::random_location(std::mt19937& rng) const
{
    std::uniform_int_distribution<int> dist(0, cols * rows - 1);
    int loc = dist(rng);
    while (types[loc] == "Obstacle")
        loc = dist(rng);
    return loc;
}
