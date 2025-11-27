#pragma once
#include "simulator.h"
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <thread>
#include <unordered_map>

class UAVTelemetryServer {
private:
	int listen_port; // current portx
	int target_port; // of Rust server if needed
	int socketfd;    // file descriptor for open socket
	bool running;
	std::unordered_map<std::string, nlohmann::json> json_pkg;
	std::thread server_thread;
	std::thread sender_thread;
	std::mutex telemetry_mutex;

public:
	// constructor
	UAVTelemetryServer(int lp = -1, int tp = 6000) :
	listen_port(lp), target_port(tp), socketfd(-1), running(false) {

		struct sockaddr_in addr;
		socklen_t addr_size = sizeof(addr);

		memset(&addr, 0, addr_size);
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = INADDR_ANY;
		addr.sin_port = htons (lp == -1 ? 0 : lp);

		socketfd = socket(AF_INET, SOCK_DGRAM, 0);
		if (socketfd < 0)
			throw std::runtime_error("Socket creation failed in UAVTelemetryServer constructor");

		if (bind(socketfd, (struct sockaddr*)&addr, addr_size) < 0)
		{
			close(socketfd);
			throw std::runtime_error("Bind failed in UAVTelemetryServer constructor.");
		}

		if (lp == -1)
		{
			if (getsockname(socketfd, (struct sockaddr*)&addr, &addr_size) < 0)
			{
				close(socketfd);
				throw std::runtime_error("Failed to get assigned port number in UAVTelemetryServer Constructor.");
			}
			listen_port = ntohs(addr.sin_port);
		}
	};

	// destructor
	~UAVTelemetryServer();

	// getter
	int get_port() { return listen_port; }
	int get_target_port() { return target_port; }
	int get_socketfd() { return socketfd; }
	std::unordered_map<std::string, nlohmann::json> get_json_pkg() {return json_pkg; }

	// setter
	void set_port(int p) { listen_port = p; }
	void set_target_port(int p) { target_port = p; }
	// void set_socketfd(int fd) { socketfd = fd; } // not sure if desired
	void update_json_pkg(const char *json_str, const struct sockaddr_in& client);

	// communications
	int start_server();
	void stop_server();
	int json_to_rust(std::string json);
	int json_from_rust();

private:
	void listen_loop();
	void sender_loop();
	void process_telemetry(const char *json_str, const struct sockaddr_in& client);
	std::string convert_json_pkg_to_string_of_array();
};