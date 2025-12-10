#pragma once
#include "environment.h"
#include <unordered_map>
#include <queue>

#define ROOT2 1.414
#define ROOT3 1.732

class Pathfinder {
private:
	Environment& env;
	int nx, ny, nz;
	double res;
	double epsilon = 1e-3;	//for simplifying actions

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

	std::vector<std::array<double, 3>> plan(
		const std::array<double, 3>& worldStart,
		const std::array<double, 3>& worldGoal
	);

	// Constructor
	Pathfinder(Environment& e) : env(e), nx(e.getNx()), ny(e.getNy()), nz(e.getNz()), res(e.getResolution()) {}

	// getter
	double getResolution() { return res; }

	// setter
	void setEpsilon(double epsilon_) { epsilon = epsilon_; }

private:
	// The 6 neighbor offsets of a cell (might expand to the 26)
	static constexpr std::array<std::array<int, 3>, 26> nbrs = {{
		// Face neighbors (cost = 1.0)
		{{1, 0, 0}}, {{-1, 0, 0}},
		{{0, 1, 0}}, {{0 , -1, 0}},
		{{0, 0, 1}}, {{0, 0, -1}},

		// Edge neighbors (cost = sqrt(2) ≈ 1.414)
		{{1, 1, 0}}, {{1, -1, 0}}, {{-1, 1, 0}}, {{-1, -1, 0}},
		{{1, 0, 1}}, {{1, 0, -1}}, {{-1, 0, 1}}, {{-1, 0, -1}},
		{{0, 1, 1}}, {{0, 1, -1}}, {{0, -1, 1}}, {{0, -1, -1}},

		// Corner neighbors (cost = sqrt(3) ≈ 1.732)
		{{1, 1, 1}}, {{1, 1, -1}}, {{1, -1, 1}}, {{1, -1, -1}},
		{{-1, 1, 1}}, {{-1, 1, -1}}, {{-1, -1, 1}}, {{-1, -1, -1}}
	}};

	inline int toIdx(int i, int j, int k) const {
		return (k * ny + j) * nx + i;
	}
	inline std::array<int, 3> toIJK(int idx) const;
	double heuristic(int idx_a, int idx_b) const;
	bool isLineClear(const std::array<double, 3>& A, const std::array<double, 3>& B) const;
	std::vector<int> rawAStar(std::array<double, 3> worldStart, std::array<double, 3> worldGoal);
	std::vector<std::array<double, 3>> smoothPath(const std::vector<int>& raw);

	void print_idx_path(std::vector<int> path);
	void print_xyz_path(std::vector<std::array<double, 3>> path);
	std::vector<std::array<double, 3>> flatArrayToWorldArray(const std::vector<int>& raw);

	double getMoveCost(const std::array<int, 3>& move) const;
};
