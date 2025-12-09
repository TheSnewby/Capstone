#include "swarm_tuning.h"
#include <mutex>

static SwarmTuning g_swarm_tuning{
	1.0,  // cohesion
	10.0, // separation
	1.0,  // alignment
	5.0,  // max_speed
	20.0, // target_altitude
	9	  // swarm_size
};

static std::mutex g_tuning_mutex;

SwarmTuning get_swarm_tuning()
{
	std::lock_guard<std::mutex> lock(g_tuning_mutex);
	return g_swarm_tuning;
}

void set_swarm_tuning(const SwarmTuning &tuning)
{
	std::lock_guard<std::mutex> lock(g_tuning_mutex);
	g_swarm_tuning = tuning;
}
