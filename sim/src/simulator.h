#pragma once
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <chrono>
#include <random>
#include <string>
#include <iomanip>

class UAV; 

enum formation {
	RANDOM = 0,
	LINE, 
	FLYING_V,
	CIRCLE,
};

class UAVSimulator {
private:
	std::vector<UAV> swarm;
	std::mutex swarm_mutex;
	std::atomic<bool> running{false};
	formation form;
	std::thread physics_thread;
	std::thread command_listener_thread;
	std::atomic<bool> command_listener_running{false};
	int command_port = 6001;

public:
	UAVSimulator(int num_drones);
	~UAVSimulator();

	//getter
	std::vector<UAV>& get_swarm() { return swarm; }
	formation get_formation() { return form; }

	// setters
	void set_formation(formation f) { form = f; }

	// methods
	void start_sim();
	void stop_sim();

	void print_swarm_status(); /* for testing */
	void change_formation(formation f);

	void start_command_listener();
	void stop_command_listener();

private:
	void physics_loop();

	void create_formation_random(int num_uavs); // default creation
	void create_formation_line(int num_uavs);	// not sure if will be used
	void create_formation_vee(int num_uavs);	// not sure if will be used
	void create_formation_circle(int num_uavs);	// not sure if will be used

	void set_formation_line(int num_uavs);
	void set_formation_vee(int num_uavs);
	void set_formation_circle(int num_uavs);

	void command_listener_loop();
};