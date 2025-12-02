#pragma once
#include "simulator.h"

// Currently Centralized
// Might need to be converted into a struct in uav.h or sim.h

class SwarmCoordinator {
private:
	// cohesion sliders
	double cohesion;
	double separation;
	double alignment;
	double max_speed;
	double target_altitude;

	std::vector<std::array<double, 3>> formation_offsets;

public:
	SwarmCoordinator() {
		cohesion = 1.00;
		separation = 12.00; // default spacing
		alignment = 1.00;
		max_speed = 30;
		target_altitude = 150;
	};

	// getters
	double get_cohesion() { return cohesion; }
	double get_separation() { return separation; }
	double get_alignment() { return alignment; }
	double get_max_speed() { return max_speed; }
	double get_target_altitude() { return target_altitude;}
	std::array<double, 3> get_formation_offset(int uav_id);

	// updaters
	void set_cohesion(double coh) { cohesion = coh; }
	void set_separation(double sep) { separation = sep; }
	void set_alignment(double align) { alignment = align; }
	void set_max_speed(double max) { max_speed = max; }
	void set_target_altitude(double target) { target_altitude = target; }
	std::array<double, 3> rotate_offset_3d(
		const std::array<double, 3>& offset,
		const std::array<double, 3>& leader_velocity
	);

	void calculate_formation_offsets(int num_uavs, formation f);
};