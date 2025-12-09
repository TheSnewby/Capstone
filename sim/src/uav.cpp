#include "uav.h"
#include "swarm_coordinator.h"
#include "swarm_tuning.h"

void UAV::update_position(double dt)
{
	pos[0] += vel[0] * dt;
	pos[1] += vel[1] * dt;
	pos[2] += vel[2] * dt;
};
void UAV::remove_neighbor_address(const std::string &address)
{
	auto addr_id = std::find(neighbors_address.begin(), neighbors_address.end(), address);
	if (addr_id != neighbors_address.end())
	{
		neighbors_address.erase(addr_id);
	}
};
void UAV::update_neighbor_status(int neighbor_id, const std::array<double, 3> &pos, const std::array<double, 3> &vel)
{
	auto now = std::chrono::steady_clock::now();

	for (auto &neighbor : neighbors_status)
	{
		if (neighbor.id == neighbor_id)
		{
			neighbor.last_known_pos = pos;
			neighbor.last_known_vel = vel; // keep velocity in sync too
			neighbor.last_time = now;
			return;
		}
	}

	// New neighbor: store both position and velocity
	neighbors_status.push_back({neighbor_id, pos, vel, now});
}

void UAV::remove_stale_neighbors()
{
	std::chrono::milliseconds max_age = std::chrono::milliseconds(1000);
	auto now = std::chrono::steady_clock::now();

	neighbors_status.erase(
		std::remove_if(neighbors_status.begin(), neighbors_status.end(),
					   [now, max_age](const NeighborInfo &neighbor)
					   {
						   return (now - neighbor.last_time) > max_age;
					   }),
		neighbors_status.end());
}

std::vector<UAV::NeighborInfo> UAV::get_fresh_neighbors()
{
	std::chrono::milliseconds max_age = std::chrono::milliseconds(500);
	std::vector<UAV::NeighborInfo> fresh_neighbors;
	auto now = std::chrono::steady_clock::now();

	for (const auto &neighbor : neighbors_status)
	{
		if ((now - neighbor.last_time) <= max_age)
			fresh_neighbors.push_back(neighbor);
	}
	return (fresh_neighbors);
}

void UAV::uav_telemetry_broadcast()
{
	// createa a json string
	std::string json_str;
	auto now = std::chrono::system_clock::now();
	std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	std::tm tm = *std::gmtime(&now_c);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
	std::string timestamp = oss.str();

	nlohmann::json j = {
		{"id", (uint64_t)get_id()},
		{"position", {{"x", pos[0]}, {"y", pos[1]}, {"z", pos[2]}}},
		{"velocity", {{"vx", vel[0]}, {"vy", vel[1]}, {"vz", vel[2]}}},
		{"timestamp", timestamp}};
	json_str = j.dump();

	std::cout << "JSON to UAVs: " << json_str << "\n";

	// open a stream to a port
	// send through stream
	// close stream
}

void UAV::uav_to_telemetry_server(int port = 6000)
{
	// createa a json string
	std::string json_str;
	auto now = std::chrono::system_clock::now();
	std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	std::tm tm = *std::gmtime(&now_c);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
	std::string timestamp = oss.str();

	nlohmann::json j = {
		{"id", get_id()},
		{"position", {{"x", pos[0]}, {"y", pos[1]}, {"z", pos[2]}}},
		{"velocity", {{"vx", vel[0]}, {"vy", vel[1]}, {"vz", vel[2]}}},
		{"timestamp", timestamp}};
	json_str = j.dump();
	// std::cout << "JSON to Telemetry Server: " << json_str << "\n";

	// open a stream to a port
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

	// send to telemetry server
	sendto_return = sendto(socketfd, json_str.c_str(), json_size, 0, (struct sockaddr *)&addr, sizeof(addr));
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

/**
 * calculate_formation_force - calculates cohesion net force towards center of uav mass
 * Return: cohesion force
 */
std::array<double, 3> UAV::calculate_formation_force()
{
	// If we have no neighbor information, we cannot compute a meaningful formation force
	std::vector<NeighborInfo> neighbors = get_neighbors_status();
	if (neighbors.empty())
	{
		return {0.0, 0.0, 0.0};
	}

	// Find the leader (id == 0) from neighbor info; fall back to the first neighbor if needed
	std::array<double, 3> leader_pos = neighbors[0].last_known_pos;
	std::array<double, 3> leader_vel = neighbors[0].last_known_vel;
	bool leader_found = false;

	for (const auto &n : neighbors)
	{
		if (n.id == 0)
		{
			leader_pos = n.last_known_pos;
			leader_vel = n.last_known_vel;
			leader_found = true;
			break;
		}
	}

	if (!leader_found)
	{
		std::cout << "WARN: leader (id 0) not found in neighbors in calculate_formation_force()" << std::endl;
	}

	// Normalize leader velocity to get a clean heading vector for rotation
	double speed = std::sqrt(
		leader_vel[0] * leader_vel[0] +
		leader_vel[1] * leader_vel[1] +
		leader_vel[2] * leader_vel[2]);

	if (speed < 1e-6)
	{
		// If the leader is effectively stationary, assume a default heading along +Y
		leader_vel = {0.0, 1.0, 0.0};
	}
	else
	{
		leader_vel[0] /= speed;
		leader_vel[1] /= speed;
		leader_vel[2] /= speed;
	}

	// Local formation offset for this UAV (defined by the formation type: LINE, VEE, CIRCLE)
	std::array<double, 3> formation_offset = SwarmCoord.get_formation_offset(get_id());

	// Use SwarmCoordinator's 3D rotation helper to map local offsets into world space
	std::array<double, 3> rotated_offset = SwarmCoord.rotate_offset_3d(formation_offset, leader_vel);

	// Target location within formation in relation to leader's location
	std::array<double, 3> formation_target = {
		leader_pos[0] + rotated_offset[0],
		leader_pos[1] + rotated_offset[1],
		leader_pos[2] + rotated_offset[2]};

	// position error
	std::array<double, 3> formation_error = {
		formation_target[0] - get_x(),
		formation_target[1] - get_y(),
		formation_target[2] - get_z()};

	// formation control parameters
	double formation_gain = 0.15;	  // proportional position gain
	double formation_force_cap = 2.0; // limit on output magnitude

	// scale position error with proportional position gain
	std::array<double, 3> formation_command = {
		formation_gain * formation_error[0],
		formation_gain * formation_error[1],
		formation_gain * formation_error[2],
	};

	// compute magnitude and apply control parameters if necessary
	double command_magnitude = std::sqrt(
		(formation_command[0] * formation_command[0]) +
		(formation_command[1] * formation_command[1]) +
		(formation_command[2] * formation_command[2]));
	if (command_magnitude > formation_force_cap && command_magnitude > 1e-6)
	{
		double scale = formation_force_cap / command_magnitude;
		formation_command[0] *= scale;
		formation_command[1] *= scale;
		formation_command[2] *= scale;
	}

	return formation_command;
}

/**
 * calculate_separation_forces - calculates separation net force away from neighbors that are too close
 * Return: separation force
 */
std::array<double, 3> UAV::calculate_separation_forces()
{
	std::array<double, 3> separation_force = {0, 0, 0};
	std::array<double, 3> current_pos = get_pos();
	std::array<double, 3> distance_arr;
	std::array<double, 3> repel_direction = {0, 0, 0};
	double repel_strength;
	double normalized_separation_direction;
	double epsilon = .001; // constant to reduce chance of division by zero
	std::vector<NeighborInfo> neighbors = get_neighbors_status();
	int num_neighbors = neighbors.size();
	double min_separation = SwarmCoord.get_separation();

	for (int i = 0; i < num_neighbors; i++)
	{
		// calculate distance between uavs
		distance_arr[0] = current_pos[0] - neighbors[i].last_known_pos[0];
		distance_arr[1] = current_pos[1] - neighbors[i].last_known_pos[1];
		distance_arr[2] = current_pos[2] - neighbors[i].last_known_pos[2];

		normalized_separation_direction = sqrt((distance_arr[0] * distance_arr[0]) + (distance_arr[1] * distance_arr[1]) + (distance_arr[2] * distance_arr[2]));
		// std::cout << "normalized_separation_direction: " << normalized_separation_direction << std::endl;

		// check if within preferred separation distance
		if (normalized_separation_direction < min_separation)
		{
			repel_strength = 1.0 / (normalized_separation_direction + epsilon);

			separation_force[0] += (distance_arr[0] / normalized_separation_direction) * repel_strength;
			separation_force[1] += (distance_arr[1] / normalized_separation_direction) * repel_strength;
			separation_force[2] += (distance_arr[2] / normalized_separation_direction) * repel_strength;
		}
	}

	return (separation_force);
}

/**
 * calculate_alignment_forces - calculates alignment net force towards the direction of the group flight
 * Return: alignment force
 */
std::array<double, 3> UAV::calculate_alignment_forces()
{
	std::array<double, 3> alignment_force = {0, 0, 0};
	std::array<double, 3> sum_of_velocities = {0, 0, 0};
	std::array<double, 3> average_velocity = {0, 0, 0};
	std::vector<NeighborInfo> neighbors_status = get_neighbors_status();
	int num_neighbors = neighbors_status.size();

	// calculate average velocity of all neighbors
	for (int i = 0; i < num_neighbors; i++)
	{
		sum_of_velocities[0] += neighbors_status[i].last_known_vel[0];
		sum_of_velocities[1] += neighbors_status[i].last_known_vel[1];
		sum_of_velocities[2] += neighbors_status[i].last_known_vel[2];
	}

	average_velocity[0] = sum_of_velocities[0] / num_neighbors / 2;
	average_velocity[1] = sum_of_velocities[1] / num_neighbors / 2;
	average_velocity[2] = sum_of_velocities[2] / num_neighbors / 2;

	// calculate alignment_force
	alignment_force[0] = average_velocity[0] - get_velx();
	alignment_force[1] = average_velocity[1] - get_vely();
	alignment_force[2] = average_velocity[2] - get_velz();

	return (alignment_force);
}

/**
 * apply_boids_forces - applies boids forces to the heading and velocity of the uav
 */
void UAV::apply_boids_forces()
{
	double internal_formation_weight = 1.5;
	double internal_separation_weight = 2.0;
	double internal_alignment_weight = 0.3; // alignment is mostly redundant and may be fully phased out in the future

	SwarmTuning tuning = get_swarm_tuning();
	double cohesion_weight = tuning.cohesion;
	double separation_weight = tuning.separation;
	double alignment_weight = tuning.alignment;
	double max_speed = tuning.max_speed;
	double target_altitude = tuning.target_altitude;
	double swarm_size = tuning.swarm_size;

	std::array<double, 3> formation_force = calculate_formation_force();

	// skip formation forces for the first few timesteps so the swarm doesn't explode on spawn
	static int formation_bootstrap_steps = 0;
	if (formation_bootstrap_steps < 5)
	{
		formation_force = {0.0, 0.0, 0.0};
		++formation_bootstrap_steps;
	}

	std::array<double, 3> separation_force = calculate_separation_forces();
	std::array<double, 3> alignment_force = calculate_alignment_forces();
	std::array<double, 3> net_force;
	std::array<double, 3> new_velocity;
	std::array<double, 3> current_velocity = get_vel();

	double fmag = sqrt(formation_force[0] * formation_force[0] +
					   formation_force[1] * formation_force[1] +
					   formation_force[2] * formation_force[2]);
	double smag = sqrt(separation_force[0] * separation_force[0] +
					   separation_force[1] * separation_force[1] +
					   separation_force[2] * separation_force[2]);
	double amag = sqrt(alignment_force[0] * alignment_force[0] +
					   alignment_force[1] * alignment_force[1] +
					   alignment_force[2] * alignment_force[2]);
	// if (get_id() == 1)
	// 	std::cout << "F="<<fmag<<" S="<<smag<<" A="<<amag<<std::endl;

	std::array<double, 3> net_formation_force = {
		(formation_force[0] * cohesion_weight * internal_formation_weight),
		(formation_force[1] * cohesion_weight * internal_formation_weight),
		(formation_force[2] * cohesion_weight * internal_formation_weight)};

	std::array<double, 3> net_separation_force = {
		(separation_force[0] * separation_weight * internal_separation_weight),
		(separation_force[1] * separation_weight * internal_separation_weight),
		(separation_force[2] * separation_weight * internal_separation_weight)};

	std::array<double, 3> net_alignment_force = {
		(alignment_force[0] * alignment_weight * internal_alignment_weight),
		(alignment_force[1] * alignment_weight * internal_alignment_weight),
		(alignment_force[2] * alignment_weight * internal_alignment_weight)};

	net_force[0] = (formation_force[0] * cohesion_weight * internal_formation_weight) + (separation_force[0] * separation_weight * internal_separation_weight) + (alignment_force[0] * alignment_weight * internal_alignment_weight);
	net_force[1] = (formation_force[1] * cohesion_weight * internal_formation_weight) + (separation_force[1] * separation_weight * internal_separation_weight) + (alignment_force[1] * alignment_weight * internal_alignment_weight);

	// Altitude control: gently push Z toward target_altitude
	double altitude_error = target_altitude - get_z();
	double altitude_gain = 0.05; // tune this to make response snappier or smoother
	double altitude_force = altitude_gain * altitude_error;

	net_force[2] = (formation_force[2] * cohesion_weight * internal_formation_weight) +
				   (separation_force[2] * separation_weight * internal_separation_weight) +
				   (alignment_force[2] * alignment_weight * internal_alignment_weight) +
				   altitude_force;

	new_velocity[0] = current_velocity[0] + net_force[0] * UAVDT;
	new_velocity[1] = current_velocity[1] + net_force[1] * UAVDT;
	new_velocity[2] = current_velocity[2] + net_force[2] * UAVDT;
	// std::cout << get_id() << ": new vx " << new_velocity[0] << " = " << current_velocity[0] << " + " << net_force[0] << " * UAVDT(" << UAVDT << ")" <<std::endl;
	// std::cout << get_id() << ": new vy " << new_velocity[1] << " = " << current_velocity[1] << " + " << net_force[1] << " * UAVDT(" << UAVDT << ")" <<std::endl;
	// std::cout << get_id() << ": new vz " << new_velocity[2] << " = " << current_velocity[2] << " + " << net_force[2] << " * UAVDT(" << UAVDT << ")" <<std::endl;

	// std::cout << "max speed: " << max_speed <<std::endl;
	new_velocity[0] = (new_velocity[0] <= max_speed) ? new_velocity[0] : max_speed;
	new_velocity[1] = (new_velocity[1] <= max_speed) ? new_velocity[1] : max_speed;
	new_velocity[2] = (new_velocity[2] <= max_speed) ? new_velocity[2] : max_speed;
	// std::cout << "Actual new_velocity[0]: " << new_velocity[0] << std::endl;
	// std::cout << "Actual new_velocity[1]: " << new_velocity[1] << std::endl;
	// std::cout << "Actual new_velocity[2]: " << new_velocity[2] << std::endl;

	// DEBUG PRINTOUT:

	/*
	std::cout << "UAV " << get_id() << " forces: "
	<< "formation(" << net_formation_force[0] << "," << net_formation_force[1] << "," << net_formation_force[2] << ") "
	<< "separation(" << net_separation_force[0] << "," << net_separation_force[1] << "," << net_separation_force[2] << ") "
	<< "alignment(" << net_alignment_force[0] << "," << net_alignment_force[1] << "," << net_alignment_force[2] << ") "
	<< "net(" << net_force[0] << "," << net_force[1] << "," << net_force[2] << ") "
	<< "New Velocity(" << new_velocity[0] << ", " << new_velocity[1] << ", " << new_velocity[2] << ")" << std::endl;
	// */
	set_velocity(new_velocity[0], new_velocity[1], new_velocity[2]);
}
