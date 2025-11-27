#include "telemetry_server.h"

UAVTelemetryServer::~UAVTelemetryServer() {
	if (socketfd > -1)
	{
		close(socketfd);
		socketfd = -1;
	}
}

/**
 * start_server - start UAV Telemetry Server
 * Return: 1 on success, 0 on failure
 */
int UAVTelemetryServer::start_server() {
	running = true;
	std::cout << "Starting listening server on port " << listen_port << "." << std::endl;
	std::cout << "Starting sender server on port " << target_port << "." <<std::endl;
	server_thread = std::thread(&UAVTelemetryServer::listen_loop, this);
	sender_thread = std::thread(&UAVTelemetryServer::sender_loop, this);

	return (1);
}

/**
 * stop_server - stops the UAV Telemetry Server
 */
void UAVTelemetryServer::stop_server() {
	running = false;
	if (server_thread.joinable())
		server_thread.join();
	if (socketfd > 0)
	{
		close(socketfd);
		socketfd = -1;
	}
}

void UAVTelemetryServer::listen_loop() {
	char buffer[4096] = {0};
	struct sockaddr_in client_addr;
	socklen_t client_size = sizeof(client_addr);

	memset(&client_addr, 0, client_size);

	while (running) {
		ssize_t bytes_recvd = recvfrom(socketfd, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&client_addr, &client_size);
		if (bytes_recvd > 0)
		{
			buffer[bytes_recvd] = '\0';
			update_json_pkg(buffer, client_addr);
		}
	}
}

/**
 * sender_loop - updates the Rust server every x ms
 */
void UAVTelemetryServer::sender_loop() {
	int update_rate = 100; // 10 Hz, adjustable currently

	while (running) {
		std::string json_pkg_as_string;

		json_pkg_as_string = convert_json_pkg_to_string_of_array();
		json_to_rust(json_pkg_as_string);

		std::this_thread::sleep_for(std::chrono::milliseconds(update_rate));
	}
}

/**
 * convert_json_pkg_to_string_of_array - converts json_pkg to a string from an array format
 */
std::string UAVTelemetryServer::convert_json_pkg_to_string_of_array() {
	std::lock_guard<std::mutex> lock(telemetry_mutex);

	nlohmann::json array = nlohmann::json::array();

	for (const auto& [id, data] : json_pkg)
		array.push_back(data);

	return (array.dump());
}
/**
 * update_json_pkg - constantly listens for incoming json from uavs
 */
void UAVTelemetryServer::update_json_pkg(const char *json_str, const struct sockaddr_in& client) {
	nlohmann::json telemetry;
	std::string id;

	try {
		telemetry = nlohmann::json::parse(json_str);

		if (telemetry.contains("id")) {
			id = telemetry["id"];

			std::lock_guard<std::mutex> lock(telemetry_mutex);
			json_pkg[id] = telemetry;
		}
	} catch (const std::exception& e) {
		std::cout << "Invalid JSON received in update_json_pkg in UAVTelemetryServer: " << e.what() << std::endl;
	}
}

/**
 * json_to_rust - sends the json_pkg to the rust server
 * Return: 1 if successful, 0 if not
 */
int UAVTelemetryServer::json_to_rust(std::string json) {
	int socketfd;
	ssize_t sendto_return = 0, json_size;
	struct sockaddr_in addr;

	socketfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketfd < 0)
	{
		std::cout << "DEBUG: failed to create UDP socket in json_to_rust" << std::endl;
		return 0;
	}

	json_size = json.length();

	const char *host_env = std::getenv("SKYWEAVE_UDP_HOST");
	const char *host = host_env ? host_env : "127.0.0.1";

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(target_port);

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
		return 0;
	}

	auto *addr_in = reinterpret_cast<sockaddr_in *>(res->ai_addr);
	addr.sin_addr = addr_in->sin_addr;
	freeaddrinfo(res);

	// send to Rust UDP listener
	sendto_return = sendto(socketfd, json.c_str(), json_size, 0, (struct sockaddr *)&addr, sizeof(addr));
	if (sendto_return == -1)
	{
		std::cout << "DEBUG: sendto in json_to_rust returned -1" << std::endl;
		close(socketfd);
		return 0;
	}
	if (sendto_return != json_size)
	{
		std::cout << "DEBUG: sendto in json_to_rust sent size mismatch" << std::endl;
		close(socketfd);
		return 0;
	}

	close(socketfd);
	return 1;
}

/**
 * jston_from_rust - receives the json_pkg from the rust server
 */
int UAVTelemetryServer::json_from_rust() {
	return (1);
}