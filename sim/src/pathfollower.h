#pragma once
#include "uav.h"
#include <vector>
#include <array>
#include <cmath>
#include <thread>

class UAV; // forward declaration to avoid circular header dependencies

class Pathfollower {
private:
	UAV& leader;
	std::vector<std::array<double, 3>> path;
	int currentIndex = 0;
	double lookahead = 10.0;
	double tolerance;

public:
	Pathfollower(UAV& leader_, double resolution) : leader(leader_), tolerance(resolution) {};

	void update_leader_velocity(double dt);

	//setter
	void setPath(const std::vector<std::array<double, 3>>& waypoints);
	void setLookahead(double lookahead_)	{ lookahead = lookahead_; }
	void setTolerance(double tolerance_)	{ tolerance = tolerance_; }

private:
	std::array<double, 3> computeCarrot() const;
};