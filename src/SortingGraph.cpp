#include "SortingGraph.h"
#include <fstream>
#include <boost/tokenizer.hpp>
#include "StateTimeAStar.h"
#include <sstream>
#include <random>
#include <chrono>

bool SortingGrid::load_map(std::string fname)
{
	std::ifstream myfile(fname);
	if (!myfile.is_open()) {
		std::cout << "Map file " << fname << " does not exist." << std::endl;
		return false;
	}

	std::string line;
	std::string token;

	getline(myfile, line); // ignore "type octile"
	getline(myfile, line);
	int height = std::stoi(line.substr(line.find(' ') + 1));
	getline(myfile, line);
	int width = std::stoi(line.substr(line.find(' ') + 1));

	this->rows = height;
	this->cols = width;

	// Setup movement directions (4-way)
	move[0] = 1;         // right
	move[1] = -cols;     // up
	move[2] = -1;        // left
	move[3] = cols;      // down

	// Read "map"
	getline(myfile, line);

	// Initialize data containers
	this->types.resize(rows * cols);
	this->weights.resize(rows * cols);

	for (int r = 0; r < rows; ++r) {
		getline(myfile, line);
		for (int c = 0; c < cols; ++c) {
			int idx = r * cols + c;
			char tile = line[c];
			types[idx] = tile;

			// Assign basic weights
			double cost = 1.0;
			weights[idx].resize(5, cost);
		}
	}

	myfile.close();
	std::size_t pos = fname.rfind('.');
	map_name = fname.substr(0, pos);
	return true;
}


void SortingGrid::preprocessing(bool consider_rotation)
{
	// std::cout << "*** PreProcessing map ***" << std::endl;
	// clock_t t = std::clock();
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
		// ensure that the heuristic table is correct
		for (auto h : heuristics)
		{
			if (types[h.first] != "Induct" && types[h.first] != "Eject")
			{
				cout << "The heuristic table does not match the map!" << endl;
				exit(-1);
			}
		}
	}
	if (!succ)
	{
		for (auto induct : inducts)
		{
			heuristics[induct.second] = compute_heuristics(induct.second);
		}
		for (auto eject_station : ejects)
		{
			for (int eject : eject_station.second)
			{
				heuristics[eject] = compute_heuristics(eject);
			}
		}
		save_heuristics_table(fname);
	}

	// double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	// std::cout << "Done! (" << runtime << " s)" << std::endl;
}
