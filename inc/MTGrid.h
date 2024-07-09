#pragma once
#include "BasicGraph.h"


class MTGrid :
	public BasicGraph
{
public:
    bool load_map(string fname);
    void preprocessing(bool consider_rotation); // compute heuristics
};
