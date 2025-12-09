#pragma once

struct SwarmTuning
{
	double cohesion;
	double separation;
	double alignment;
	double max_speed;
	double target_altitude;
	int swarm_size;
};

SwarmTuning get_swarm_tuning();
void set_swarm_tuning(const SwarmTuning &tuning);
