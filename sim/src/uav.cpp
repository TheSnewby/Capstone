#include "uav.h"


void UAV::update_position(double dt) {
	pos[0] += vel[0] * dt;
	pos[1] += vel[1] * dt;
	pos[2] += vel[2] * dt; 
};
void UAV::remove_neighbor_address(const std::string& address) {
	auto addr_id = std::find(neighbor_address.begin(), neighbor_address.end(), address);
	if (addr_id != neighbor_address.end()) { neighbor_address.erase(addr_id); }
};
void UAV::update_neighbor_status(int neighbor_id, const std::array<double, 3>& pos) {
	auto now = std::chrono::steady_clock::now();

	for (auto& neighbor : neighbor_status) {
		if (neighbor.id == neighbor_id) {
			neighbor.last_known_pos = pos;
			neighbor.last_time = now;
			return;
		}
	}
	neighbor_status.push_back({neighbor_id, pos, now});
}

void UAV::remove_stale_neighbors() {
	std::chrono::milliseconds max_age = std::chrono::milliseconds(1000);
	auto now = std::chrono::steady_clock::now();

	neighbor_status.erase(
		std::remove_if(neighbor_status.begin(), neighbor_status.end(),
		[now, max_age](const NeighborInfo& neighbor) {
			return (now - neighbor.last_time) > max_age;
		}),
		neighbor_status.end()
	);
}

std::vector<UAV::NeighborInfo> UAV::get_fresh_neighbors() {
	std::chrono::milliseconds max_age = std::chrono::milliseconds(500);
	std::vector<UAV::NeighborInfo> fresh_neighbors;
	auto now = std::chrono::steady_clock::now();

	for (const auto& neighbor: neighbor_status) {
		if ((now - neighbor.last_time) <= max_age)
			fresh_neighbors.push_back(neighbor);
	}
	return (fresh_neighbors);
}

void UAV::uav_telemetry_broadcast() {
	//createa a json string
	std::string json_str;
	auto now = std::chrono::system_clock::now();
	std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	std::tm tm = *std::gmtime(&now_c);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
	std::string timestamp = oss.str();

	nlohmann::json j = {
	    { "id", get_id() },
	    { "position", {
	        { "x", pos[0] },
	        { "y", pos[1] },
	        { "z", pos[2] }
	    }},
	    { "velocity", {
	        { "vx", vel[0] },
	        { "vy", vel[1] },
	        { "vz", vel[2] }
	    }},
	    { "timestamp", timestamp }
	};
	json_str = j.dump();

	std::cout << "JSON to UAVs: " << json_str << "\n";

	//open a stream to a port
	//send through stream
	//close stream
}

void UAV::uav_to_telemetry_server(int port = 6000) {
	//createa a json string
	std::string json_str;
	auto now = std::chrono::system_clock::now();
	std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	std::tm tm = *std::gmtime(&now_c);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
	std::string timestamp = oss.str();

	nlohmann::json j = {
	    { "id", get_id() },
	    { "position", {
	        { "x", pos[0] },
	        { "y", pos[1] },
	        { "z", pos[2] }
	    }},
	    { "velocity", {
	        { "vx", vel[0] },
	        { "vy", vel[1] },
	        { "vz", vel[2] }
	    }},
	    { "timestamp", timestamp }
	};
	json_str = j.dump();
	std::cout << "JSON to Telemetry Server: " << json_str << "\n";

	//open a stream to a port
	int socketfd;
	ssize_t sendto_return = 0, json_size;
	struct sockaddr_in addr;

	socketfd = socket(AF_INET, SOCK_DGRAM, 0);
	json_size = json_str.length();

	const char *host_env = std::getenv("SKYWEAVE_UDP_HOST");
	const char *host = host_env ? host_env : "127.0.0.1";

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);

	// Resolve hostname to IPv4 address (supports DNS names like *.fly.dev)
	addrinfo hints;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;

	addrinfo *res = nullptr;
	int gai_err = getaddrinfo(host, nullptr, &hints, &res);
	if (gai_err != 0 || res == nullptr)
	{
		std::cout << "DEBUG: getaddrinfo failed for host " << host << ": " << gai_strerror(gai_err) << std::endl;
		close(socketfd);
		return;
	}

	auto *addr_in = reinterpret_cast<sockaddr_in *>(res->ai_addr);
	addr.sin_addr = addr_in->sin_addr;
	freeaddrinfo(res);

	//send to telemetry server
	sendto_return = sendto(socketfd, json_str.c_str(), json_size, 0, (struct sockaddr*)&addr, sizeof(addr));
	if (sendto_return == -1)
	{
		std::cout << "DEBUG: sendto in uav_to_telemetry_server returned -1" << std::endl;
		close(socketfd);
	}
	if (sendto_return != json_size)
	{
		std::cout << "DEBUG: sendto in uav_to_telemetry_server sent size mismatch" << std::endl;
		close(socketfd);
	}

	close(socketfd);
}