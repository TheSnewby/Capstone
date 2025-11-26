#pragma once
#include "uav.h"
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

class UAVSimulator {
private:
	std::vector<UAV> swarm;
	std::mutex swarm_mutex;
	std::atomic<bool> running{false};

	std::thread physics_thread;

public:
	UAVSimulator(int num_drones);
	~UAVSimulator();

	//getter
	std::vector<UAV>& get_swarm() { return swarm; }

	void start_sim();
	void stop_sim();

	void print_swarm_status(); /* for testing */
	void update_uav_pos(int index, int dt, int tel_serv_port);

private:
	void physics_loop();
};