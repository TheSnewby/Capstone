#include "simulator.h"
#include "uav.h"
#include <cmath>

/**
 * generate_test_obstacles - generates obstacles at set locations
 */
void UAVSimulator::generate_test_obstacles()
{
	env.addBox(-10, 10, 20, 10, 30, 60);
	env.addBox(-10, -10, 20, 10, 10, 60);
}

/**
 * RTB - Return To Base: returns leader to base
 */
void UAVSimulator::RTB()
{
	pathfinder.plan(swarm[0].get_pos(), {0.0, 0.0, 20.0});

	// change speed if at zero
	auto vel = swarm[0].get_vel();
	if (sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]) < 1e-2)
		swarm[0].set_velocity(-1,-1,-1);  // (better if set to direct course)
}

/**
 * print_swarm_status: prints all UAV's position and velocity to stdout
 */
void UAVSimulator::print_swarm_status()
{
	size_t swarm_size = swarm.size();

	std::cout << std::fixed << std::setprecision(2);
	std::cout << "\nPrinting current swarm.\n"
			  << std::endl;
	std::cout << "ID: Position X, Y, Z. Velocity: vx, vy, vz" << std::endl;

	for (size_t i = 0; i < swarm_size; i++)
	{
		std::cout << swarm[i].get_id() << ": Position " << swarm[i].get_x()
				  << ", " << swarm[i].get_y() << ", " << swarm[i].get_z()
				  << ". Velocity: " << swarm[i].get_velx() << ", " << swarm[i].get_vely()
				  << ", " << swarm[i].get_velz() << std::endl;
	}
};

/**
 * Constructor for UAVSimulator
 */
UAVSimulator::UAVSimulator(int num_uavs) : env(BORDER_X / RESOLUTION, BORDER_Y / RESOLUTION, BORDER_Z / RESOLUTION, RESOLUTION),
										   pathfinder(env)
{
	// create base UAVs at a common starting point and base altitude
	swarm.reserve(num_uavs); // allocates memory to reduce resizing slowdowns
	for (int i = 0; i < num_uavs; i++)
	{
		// leader and followers start co-located; formation offsets will spread them out
		swarm.push_back(UAV(i, 8000 + i, 0.0, 0.0, 20.0, env));
		// give everyone an initial forward velocity along +Y
		swarm[i].set_velocity(0.0, 4.0, 0.0); // cruisin on y axis
	}

	// set initial formation (LINE as default) and compute offsets
	change_formation(FLYING_V);

	// apply formation offsets around the leader so the swarm starts in formation
	if (!swarm.empty())
	{
		// use UAV with id 0 as leader
		std::size_t leader_idx = 0;
		for (std::size_t i = 0; i < swarm.size(); ++i)
		{
			if (swarm[i].get_id() == 0)
			{
				leader_idx = i;
				break;
			}
		}

		// leader position defines the origin for formation placement
		double leader_x = swarm[leader_idx].get_x();
		double leader_y = swarm[leader_idx].get_y();
		double leader_z = swarm[leader_idx].get_z();

		SwarmCoordinator &coords = swarm[leader_idx].get_SwarmCoord();

		// Place UAVs directly using formation offsets in world space.
		// Initial formation is aligned to global axes, and Z is kept at leader altitude.
		for (int i = 0; i < num_uavs; i++)
		{
			std::array<double, 3> base_offset = coords.get_formation_offset(i);

			swarm[i].set_position(
				leader_x + base_offset[0],
				leader_y + base_offset[1],
				leader_z);
		}
	}

	std::cout << "Created swarm with " << num_uavs << " UAVs" << std::endl;
	print_swarm_status();

	// Set Up Environment
	env.generate_random_obstacles(60);
	// generate_test_obstacles(); 					// for testing

	std::array<double, 3> startXYZ = swarm[0].get_pos();
	// Pick a corner goal 50m above start altitude to ensure vertical clearance
	double corner_offset = RESOLUTION * 0.5; // center of final cell inside bounds
	double corner_x = (BORDER_X / 2.0) - corner_offset;
	double corner_y = (BORDER_Y / 2.0) - corner_offset;
	goalXYZ = {corner_x, corner_y, startXYZ[2] + 50.0};
	goalRadius = 6.0;
	// mark goal for visualization (approx 3x UAV size) and store radius
	env.setGoal(goalXYZ, goalRadius);
	env.environment_to_rust(RUST_UDP_PORT);
	std::vector<std::array<double, 3>> path = pathfinder.plan(startXYZ, goalXYZ);
	pathfollower = std::make_unique<Pathfollower>(swarm[0], env.getResolution());
	pathfollower->setPath(path);
};

/**
 * ~UAVSimulator - Destructor for UAVSimulator
 */
UAVSimulator::~UAVSimulator()
{
	stop_sim();
}

/**
 * start_turn_timer - testing function with places for commands
 */
void UAVSimulator::start_turn_timer()
{
	turn_timer_thread = std::thread([this]()
									{
		std::this_thread::sleep_for(std::chrono::seconds(20));
		if (running)
			change_formation(FLYING_V);
		std::this_thread::sleep_for(std::chrono::seconds(20));
		if (running)
			change_formation(CIRCLE); });
	turn_timer_thread.detach();
}

/**
 * start_sim -	starts the simulation loop in a separate thread,
 *				updating UAV positions and sending telemetry to server
 */
void UAVSimulator::start_sim()
{
	if (running)
		return;

	std::cout<< "UINTMAX = " << UINT_MAX <<std::endl;
	std::cout<< "INTMAX = " << INT_MAX <<std::endl;

	running = true;

	// start_turn_timer();

	std::thread([this]()
				{
		using namespace std::chrono;
		const auto sleep_duration = milliseconds(int(1000 * UAVDT));    // 20 Hz Updates with .05 UAVDT
		const int telemetry_port = 6000;

		while (running) {
			for (auto &uav : swarm) {
				if (uav.get_id() == 0 && pathfollower && leader_autopilot.load()) // only drive leader when autopilot enabled
					pathfollower->update_leader_velocity(UAVDT);

				// // Apply obstacle repulsion to the leader so it diverts away from collisions
				// if (uav.get_id() == 0) {
				// 	auto obs = uav.calculate_obstacle_forces();
				// 	double mag = std::sqrt(obs[0] * obs[0] + obs[1] * obs[1] + obs[2] * obs[2]);
				// 	if (mag > 1e-6) {
				// 		const double max_delta = 3.0;
				// 		double scale = std::min(1.0, max_delta / mag);
				// 		const double gain = 0.5;
				// 		auto vel = uav.get_vel();
				// 		uav.set_velocity(
				// 			vel[0] + gain * obs[0] * scale,
				// 			vel[1] + gain * obs[1] * scale,
				// 			vel[2] + gain * obs[2] * scale
				// 		);
				// 	}
				// }

				uav.update_position(UAVDT); // UAVDT found in uav.h
				uav.uav_to_telemetry_server(telemetry_port);
			}

			// Centralized neighbors updater
			// (to be used until working and then will be decentralized)
			const int num_uav = swarm.size();
			for (int i = 0; i < num_uav; i++) {
				for (int j = 0; j < num_uav; j++) {
					if (i != j) {
						swarm[i].update_neighbor_status(
							swarm[j].get_id(),
							swarm[j].get_pos(),
							swarm[j].get_vel()
						);
					}
				}
				if (i != 0)
					swarm[i].apply_boids_forces();
			}

			// if leader reaches the goal, stop and arrange followers around the beacon
			if (!reached_goal && !swarm.empty()) {
				auto leader_pos = swarm[0].get_pos();
				double dx = leader_pos[0] - goalXYZ[0];
				double dy = leader_pos[1] - goalXYZ[1];
				double dz = leader_pos[2] - goalXYZ[2];
				double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
				if (dist <= goalRadius) {
					reached_goal = true;
					leader_autopilot.store(false);
					swarm[0].set_position(goalXYZ[0], goalXYZ[1], goalXYZ[2]);
					swarm[0].set_velocity(0.0, 0.0, 0.0);

					int followers = static_cast<int>(swarm.size()) - 1;
					if (followers > 0) {
						double ring_radius = goalRadius * 1.4;
						for (int idx = 0; idx < followers; ++idx) {
							double t = (idx + 0.5) / followers;
							double phi = std::acos(1.0 - 2.0 * t);
							double theta = M_PI * (1.0 + std::sqrt(5.0)) * idx;
							double x = ring_radius * std::sin(phi) * std::cos(theta);
							double y = ring_radius * std::sin(phi) * std::sin(theta);
							double z = ring_radius * std::cos(phi);
							int uav_idx = idx + 1;
							if (uav_idx < num_uav) {
								swarm[uav_idx].set_position(goalXYZ[0] + x, goalXYZ[1] + y, goalXYZ[2] + z);
								swarm[uav_idx].set_velocity(0.0, 0.0, 0.0);
							}
						}
					}
				}
			}
			std::this_thread::sleep_for(sleep_duration);
		} })
		.detach();
}

void UAVSimulator::stop_sim()
{
	running = false;
	stop_command_listener();
}

/**
 * change_formation - changes formation of swarm
 * @f: formation enum (1: LINE, 2: FLYING_VEE, 3: CIRCLE)
 */
void UAVSimulator::change_formation(formation f)
{
	int uav_nums = swarm.size();

	SwarmCoordinator &coords = swarm[0].get_SwarmCoord();
	coords.calculate_formation_offsets(uav_nums, f);

	// formation offsets stored in each uav
	for (int i = 0; i < uav_nums; i++)
	{
		SwarmCoordinator &uav_coord = swarm[i].get_SwarmCoord();
		uav_coord = coords;
	}

	form = f; // (could reorder to have this queue off form changes)

	if (f == 1)
	{
		std::cout << "Formation changed to LINE." << std::endl;
	}
	else if (f == 2)
	{
		std::cout << "Formation changed to FLYING VEE." << std::endl;
	}
	else if (f == 3)
	{
		std::cout << "Formation changed to CIRCLE." << std::endl;
	}
}

void UAVSimulator::resize_swarm(int new_size)
{
	if (new_size < 1)
		new_size = 1;

	// Preserve leader state if available
	double leader_x = 0.0;
	double leader_y = 0.0;
	double leader_z = 20.0;
	double leader_vx = 0.0;
	double leader_vy = 1.0;
	double leader_vz = 0.0;

	if (!swarm.empty())
	{
		std::size_t leader_idx = 0;
		for (std::size_t i = 0; i < swarm.size(); ++i)
		{
			if (swarm[i].get_id() == 0)
			{
				leader_idx = i;
				break;
			}
		}

		leader_x = swarm[leader_idx].get_x();
		leader_y = swarm[leader_idx].get_y();
		leader_z = swarm[leader_idx].get_z();
		leader_vx = swarm[leader_idx].get_velx();
		leader_vy = swarm[leader_idx].get_vely();
		leader_vz = swarm[leader_idx].get_velz();
	}

	// rebuild the swarm around the leader
	swarm.clear();
	swarm.reserve(new_size);

	for (int i = 0; i < new_size; ++i)
	{
		int uav_port = 8000 + i;
		// leader and followers start co-located; formation offsets will spread them out
		UAV uav(i, uav_port, leader_x, leader_y, leader_z, env);
		uav.set_velocity(leader_vx, leader_vy, leader_vz);
		swarm.push_back(uav);
	}

	// Recompute formation offsets for the current formation so the new swarm starts in formation
	change_formation(form);

	std::cout << "Resized swarm to " << new_size << " UAVs" << std::endl;
}

void UAVSimulator::start_command_listener()
{
	command_listener_running = true;
	command_listener_thread = std::thread(&UAVSimulator::command_listener_loop, this);
}

void UAVSimulator::stop_command_listener()
{
	command_listener_running = false;
	if (command_listener_thread.joinable())
		command_listener_thread.join();
}

void UAVSimulator::command_listener_loop()
{
	int socketfd = socket(AF_INET6, SOCK_DGRAM, 0);
	struct sockaddr_in6 addr;
	char buffer[1024] = {0};

	memset(&addr, 0, sizeof(addr));
	addr.sin6_family = AF_INET6;
	addr.sin6_addr = in6addr_any; // listen on all IPv6 interfaces (including WireGuard)
	addr.sin6_port = htons(command_port);

	if (bind(socketfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		std::cout << "Failed to bind IPv6 command listener to port " << command_port << std::endl;
		return;
	}

	std::cout << "IPv6 command listener started on port " << command_port << std::endl;

	while (command_listener_running)
	{
		ssize_t received = recvfrom(socketfd, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
		if (received <= 0)
		{
			continue;
		}

		// convert bytes received to string for parsing
		std::string command(buffer, received);

		// temp debugging output
		std::cout << "Received command: [" << command << "]" << std::endl;

		// nix any trailing shit
		while (!command.empty() &&
			   (command.back() == '\n' ||
				command.back() == '\r' ||
				command.back() == ' ' ||
				command.back() == '\0'))
		{
			command.pop_back();
		}

		if (command == "1" || command == "line")
			change_formation(LINE);
		else if (command == "2" || command == "vee")
			change_formation(FLYING_V);
		else if (command == "3" || command == "circle")
			change_formation(CIRCLE);
		else if (command.rfind("move_leader", 0) == 0)
		{
			std::stringstream ss(command);
			std::string tag;
			std::string dir;
			ss >> tag >> dir;

			// manual commands disable autopilot until explicitly re-enabled
			leader_autopilot.store(false);

			if (swarm.empty())
				continue;

			// make leader's ID 0 instead of assuming UAV at index 0 is leader
			auto find_leader_idx = [&]() {
				size_t leader_idx = 0;
				for (size_t i = 0; i < swarm.size(); i++) {
					if (swarm[i].get_id() == 0) {
						leader_idx = i;
						break;
					}
				}
				return leader_idx;
			};
			size_t leader_idx = find_leader_idx();

			// read current leader velocity
			double vx = swarm[leader_idx].get_velx();
			double vy = swarm[leader_idx].get_vely();
			double vz = swarm[leader_idx].get_velz();

			// compute speed and heading
			double speed = std::sqrt(vx * vx + vy * vy);
			double heading = std::atan2(vy, vx);

			// modify speed and heading based on command
			const double min_speed = 1e-3;
			if (speed < min_speed)
				heading = M_PI_2; // default heading if stationary

			if (dir == "accelerate")
			{
				const double delta_speed = 1.0;
				speed += delta_speed;
			}
			else if (dir == "decelerate")
			{
				const double delta_speed = 0.5;
				speed = std::max(0.0, speed - delta_speed);
			}
			else if (dir == "left")
			{
				const double delta_angle = M_PI / 36; // 5 degrees
				heading -= delta_angle;
			}
			else if (dir == "right")
			{
				const double delta_angle = M_PI / 36; // 5 degrees
				heading += delta_angle;
			}

			// compute new velocity components
			vx = speed * std::cos(heading);
			vy = speed * std::sin(heading);

			// update leader velocity
			swarm[leader_idx].set_velocity(vx, vy, vz);
		}

		// altitude change command
		else if (command.rfind("altitude_change", 0) == 0)
		{
			std::stringstream ss(command);
			std::string tag;
			double delta = 0.0;
			ss >> tag >> delta;

			if (swarm.empty())
				continue;

			size_t leader_idx = 0;
			for (size_t i = 0; i < swarm.size(); i++)
			{
				if (swarm[i].get_id() == 0)
				{
					leader_idx = i;
					break;
				}
			}

			// update leader altitude smoothly by setting a gentle vertical velocity for a short duration
			double z = swarm[leader_idx].get_z();
			double target_z = z + delta;
			double vz = (delta > 0 ? 1.0 : -1.0); // 1 m/s climb or descent
			// clamp to not overshoot
			double remaining = target_z - z;
			if ((delta > 0 && vz > remaining) || (delta < 0 && vz < remaining))
				vz = remaining;
			swarm[leader_idx].set_velocity(swarm[leader_idx].get_velx(), swarm[leader_idx].get_vely(), vz);
		}

		// return-to-base command
		else if (command == "rtb")
		{
			if (swarm.empty())
				continue;

			auto find_leader_idx = [&]() {
				size_t leader_idx = 0;
				for (size_t i = 0; i < swarm.size(); i++) {
					if (swarm[i].get_id() == 0) {
						leader_idx = i;
						break;
					}
				}
				return leader_idx;
			};
			size_t leader_idx = find_leader_idx();

			// ensure autopilot is on so RTB path is followed
			leader_autopilot.store(true);
			std::array<double, 3> start = swarm[leader_idx].get_pos();
			std::array<double, 3> base = {0.0, 0.0, 20.0};
			auto path = pathfinder.plan(start, base);
			if (path.empty()) {
				path.push_back(start);
				path.push_back(base);
			}
			if (!pathfollower) {
				pathfollower = std::make_unique<Pathfollower>(swarm[leader_idx], env.getResolution());
			}
			pathfollower->setPath(path);
			std::cout << "RTB: leader plotting path back to base" << std::endl;
		}

		// toggle leader flight mode: "flight_mode autonomous|controlled"
		else if (command.rfind("flight_mode", 0) == 0)
		{
			std::stringstream ss(command);
			std::string tag;
			std::string mode;
			ss >> tag >> mode;

			if (mode == "autonomous")
				leader_autopilot.store(true);
			else if (mode == "controlled")
				leader_autopilot.store(false);
		}

		// clear buffer for next recvfrom
		memset(buffer, 0, sizeof(buffer));
	}
}
