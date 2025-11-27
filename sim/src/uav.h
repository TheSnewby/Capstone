#pragma once
#include <array>
#include <vector>
#include <string>
#include <chrono>
#include <sstream>
#include <ctime>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <iomanip>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <cstdlib>
#include <arpa/inet.h>
#include <unistd.h>



// for the stretch goal of allowing a user to manually control an individual UAV
enum class UAVControleMode {
	AUTONOMOUS,
	MANUAL,
	// perhaps LEADER
};

class UAV {
private:
	int id;
	int port;
	std::vector<std::string> neighbor_address; /* 172.0.0.1:8001, ...*/
	UAVControleMode mode = UAVControleMode::AUTONOMOUS;

	std::array<double, 3> pos;
	std::array<double, 3> vel;

	struct NeighborInfo {
		int id;
		std::array<double, 3> last_known_pos;
		std::chrono::time_point<std::chrono::steady_clock> last_time;
	};
	std::vector<NeighborInfo> neighbor_status;

public:
	// Constructor
	UAV(int set_id, int set_port, double x, double y, double z) :
		id(set_id), port(set_port), pos{x, y, z}, vel{0, 0, 0} {}

	// Setters
	void set_velocity(double x, double y, double z) { vel = {x, y, z}; }
	void set_velx(double x) { vel[0] = x; }
	void set_vely(double y) { vel[1] = y; }
	void set_velz(double z) { vel[2] = z; }
	void set_neighbor_address(std::vector<std::string> addresses) {neighbor_address = addresses; }

	// Getters
	int get_id() const { return id; }
	int get_port() const { return port; }

	std::array<double, 3> get_pos() const { return pos; }
	double get_x() const { return pos[0]; }
	double get_y() const { return pos[1]; }
	double get_z() const { return pos[2]; }

	std::array<double, 3> get_vel() const { return vel; }
	double get_velx() const { return vel[0]; }
	double get_vely() const { return vel[1]; }
	double get_velz() const { return vel[2]; }

	std::vector<std::string> get_neighbor_address() { return neighbor_address; }
	std::vector<NeighborInfo> get_neighbor_status() { return neighbor_status; }

	// Updaters
	void update_position(double dt);
	void add_neighbor_address(const std::string& address) { neighbor_address.push_back(address); }
	void remove_neighbor_address(const std::string& address);
	void update_neighbor_status(int neighbor_id, const std::array<double, 3>& pos);
	void remove_stale_neighbors();
	std::vector<NeighborInfo> get_fresh_neighbors();

	// JSON
	void uav_telemetry_broadcast();
	void uav_to_telemetry_server(int port);
};
