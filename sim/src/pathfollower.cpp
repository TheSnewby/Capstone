#include "pathfollower.h"
#include "uav.h"

/**
 * setPath - sets the path from the passed waypoints and resets currentIndex
 * @waypoints: smooth path from pathfinder's path()
 */
void Pathfollower::setPath(const std::vector<std::array<double, 3>>& waypoints) {
	path = waypoints;
	currentIndex = 0;
}

/**
 * computeCarrot - computes a steering location lookahead meters in front of uav along path to steer towards
 *
 * Return: steering location lookahead meters along path
 */
std::array<double, 3> Pathfollower::computeCarrot() const {
	double remain = lookahead;		// remainder of lookahead
	int path_size = path.size();
	std::array<double, 3> carrot;

	// iterate through points until finding lookahead point along path
	for (int i = currentIndex; i + 1< path_size; i++) {
		const auto& A = path[i];
		const auto& B = path[i + 1];
		double dx = B[0] - A[0];
		double dy = B[1] - A[1];
		double dz = B[2] - A[2];
		double segLen = std::sqrt(dx * dx + dy * dy + dz * dz);	// length of segment between points
		if (remain < segLen) {
			double frac = remain / segLen;
			carrot = {A[0] + frac * dx, A[1] + frac * dy, A[2] + frac * dz};
			return (carrot);
		}
		remain -= segLen;	// remove segLen from remainder as looking at next point
	}
	return path.back();		// use final waypoint if lookahead exceeds path remaining
}

/**
 * update - update leader position
 */
void Pathfollower::update_leader_velocity(double dt) {
	if (path.empty())
		return;

	auto pos = leader.get_pos();
	int path_size = path.size();

	// advance currentIndex if needed including type-packed wavepoints
	while (currentIndex < path_size) {
		auto& wp = path[currentIndex];
		double dx = wp[0] - pos[0];
		double dy = wp[1] - pos[1];
		double dz = wp[2] - pos[2];

		if (std::sqrt(dx * dx + dy * dy + dz *dz) < tolerance)
			currentIndex++;
		else
			break;
	}

	if (currentIndex >= path_size)
		return;	// goal reached

	// compute carrot
	auto carrot = computeCarrot();
	double dx = carrot[0] - pos[0];
	double dy = carrot[1] - pos[1];
	double dz = carrot[2] - pos[2];

	// compute distance to carrot
	double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
	if (dist < 1e-6)
		return;

	// maintain current speed magnitude in directional change
	auto vel = leader.get_vel();
	double speed = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);
	// if (speed < 1e-3)	// a fallback if velocity detiorates to 0
	// 	speed = 1.0;

	double vx = speed * dx / dist;
	double vy = speed * dy / dist;
	double vz = speed * dz / dist;
	leader.set_velocity(vx, vy, vz);
}