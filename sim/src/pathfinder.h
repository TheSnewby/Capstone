#pragma once
#include "environment.h"
#include <unordered_map>
#include <queue>

class Pathfinder {
private:
	Environment & env;
	int nx, ny, nz;
	double res;

public:
	struct Node {
		int idx;		// flattened cell index
		double f;		// the A* key estimating total path cost via this node
		double g;		// the cost from the start node to the current node
	};
	struct NodeCmp {
		bool operator() (Node const&a, Node const& b) const {
			return a.f > b.f;
		}
	};

	// Constructor
	Pathfinder(Environment& e) : env(e), nx(e.getNx()), ny(e.getNy()), nz(e.getNz()), res(e.getResolution()) {}

private:
	// The 6 neighbor offsets of a cell (might expand to the 26)
	static constexpr std::array<std::array<int, 3>, 6> nbrs = {{
		{{1, 0, 0}}, {{-1, 0, 0}},
		{{0, 1, 0}}, {{0 , -1, 0}},
		{{0, 0, 1}}, {{0, 0, -1}}
	}};

	inline int toIdx(int i, int j, int k) const {
		return (k * ny + j) * nx + i;
	}
	inline std::array<int, 3> toIJK(int idx) const;
	double heuristic(int idx_a, int idx_b) const;
	std::vector<std::array<double, 3>> findPath (std::array<double, 3> worldStart, std::array<double, 3> worldGoal);
};
