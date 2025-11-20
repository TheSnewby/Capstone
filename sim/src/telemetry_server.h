#pragma once
#include "simulator.h"
#include <nlohmann/json.hpp>

class UAVTelemetryServer {
private:
	int port; // current port
	int target_port; // of Rust server if needed
	std::vector<std::string> json_pkg;

public:
	// constructor
	UAVTelemetryServer(int p, int tp) : port(p), target_port(tp) {};

	// getter
	int get_port() { return port; }
	int get_target_port() { return target_port; }
	std::vector<std::string> get_json_pkg() {return json_pkg; }

	// setter
	void set_port(int p) { port = p; }
	void set_target_port(int p) { target_port = p; }
	void update_json_pkg();

	// communications
	int json_to_rust();
	int json_from_rust();
};