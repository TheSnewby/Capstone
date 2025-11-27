#include "uav.h"
#include "simulator.h"
#include "telemetry_server.h"

int main() {
	int i, num_uav = 3, cycle = 0, cycles = 10;
	UAVSimulator sim(num_uav);
	std::vector<UAV>& swarm = sim.get_swarm();
	UAVTelemetryServer tel_serv(-1, 6000);

	//initialize velocity
	for (i = 0; i < num_uav; i++) {
		swarm[i].set_velocity(1.0, 1.0, 1.0);
	}

	tel_serv.start_server();

	//physics loop (will later be moved to simulator.h)
	while (cycle < cycles) {
		int time_interval = 10; // 100 Hz ~ 10ms 
		std::cout << "Cycle: " << cycle << "." << std::endl;
		for (i = 0; i < num_uav; i++)
		{
			swarm[i].update_position(time_interval);
			swarm[i].uav_to_telemetry_server(tel_serv.get_port());
		}

	std::this_thread::sleep_for(std::chrono::milliseconds(time_interval));
	sim.print_swarm_status();
	std::cout.flush();
	cycle++;
	}

	tel_serv.stop_server();

	return (0);
}