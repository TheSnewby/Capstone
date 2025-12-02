#include "uav.h"
#include "simulator.h"
#include "telemetry_server.h"
#include "swarm_coordinator.h"

int main() {
	int num_uav = 10;
	UAVSimulator sim(num_uav);
	std::vector<UAV>& swarm = sim.get_swarm();

	// initialize velocity for all UAVs
	for (int i = 0; i < num_uav; i++) {
		swarm[i].set_velocity(1.0, 1.0, 1.0);
	}

	// start the simulator's internal loop (updates + telemetry)
	sim.start_sim();

	// std::cout << "Simulation running with " << num_uav << " UAVs in V formation." << std::endl;

	// program is now an indefinite loop - must be terminated manually
	while (true) {
		sim.print_swarm_status();
		std::cout.flush();
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// here for.. just in case.
	sim.stop_sim();
	return 0;
}
