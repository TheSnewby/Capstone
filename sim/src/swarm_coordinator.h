#pragma once
#include "simulator.h"

// Currently Centralized

class SwarmCoordinator {
private:
	// cohesion sliders
	double cohesion;
	double separation;
	double alignment;
	double max_speed;
	double target_altitude;

public:
	SwarmCoordinator() {
		cohesion = separation = alignment = 1.00;
		max_speed = 30;
		target_altitude = 150;
	};

	// getters
	double get_cohesion() { return cohesion; }
	double get_separation() { return separation; }
	double get_alignment() { return alignment; }
	double get_max_speed() { return max_speed; }
	double get_target_altitude() { return target_altitude;}

	// updaters
	void set_cohesion(double coh) { cohesion = coh; }
	void set_separation(double sep) { separation = sep; }
	void set_alignment(double align) { alignment = align; }
	void set_max_speed(double max) { max_speed = max; }
	void set_target_altitude(double target) { target_altitude = target; }

};