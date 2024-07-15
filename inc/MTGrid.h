#pragma once
#include "BasicGraph.h"
#include <random>


class MTGrid :
	public BasicGraph
{
public:
    bool load_map(string fname);
    void preprocessing(bool consider_rotation); // compute heuristics
    int random_location(std::mt19937& rng) const;
};
