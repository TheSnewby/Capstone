#include "simulator.h"
#include <iostream>
#include <chrono>
#include <random>
#include <string>
#include <iomanip>

/**
 * print_swarm_status: prints all UAV's position and velocity to stdout
 */
void UAVSimulator::print_swarm_status() {
	size_t swarm_size = swarm.size();

	std::cout << std::fixed << std::setprecision(2);
	std::cout << "\nPrinting current swarm.\n" << std::endl;
	std::cout <<"ID: Position X, Y, Z. Velocity: vx, vy, vz" << std::endl;

	for (size_t i = 0; i < swarm_size; i++)
	{
		std::cout << swarm[i].get_id() << ": Position " << swarm[i].get_x() <<
		", " << swarm[i].get_y() << ", " << swarm[i].get_z() << ". Velocity: " <<
		swarm[i].get_velx() << ", " << swarm[i].get_vely() << ", " << swarm[i].get_velz() << std::endl; 
	}
};

/**
 * Constructor for UAVSimulator
 */
UAVSimulator::UAVSimulator(int num_uavs) {
	int i;
	std::random_device rd; /* randomizer */
	std::mt19937 gen(rd()); /* super randomizer */

	// generator of a super random number within a bounded uniform distribution
	std::uniform_real_distribution<> distrib(-10.0, 10.0);

	// generate a swarm of uavs
	for (i = 0; i < num_uavs; i++) {
		UAV uav (i, i * 100, distrib(gen), distrib(gen), 0);
		swarm.push_back(uav);
	}

	std::cout << "Created swarm with " << num_uavs << " UAVs" << std::endl;
	print_swarm_status();
};

/**
 * ~UAVSimulator - Destructor for UAVSimulator
 */
UAVSimulator::~UAVSimulator() {
	stop_sim();
}

/**
 * update_uav_pos - updates and broadcasts to the telemetry server
 */
void update_uav_pos(int index, int dt, int tel_serv_port) {
	
}

void UAVSimulator::start_sim() {
	running = true;

}

void UAVSimulator::stop_sim() {
	running = false;

}