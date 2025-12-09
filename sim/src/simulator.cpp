#include "simulator.h"
#include "uav.h"

/**
 * generate_test_obstacles - generates obstacles at set locations
 */
void UAVSimulator::generate_test_obstacles() {
	env.addBox(-10, 10, 20, 10, 30, 60);
	env.addBox(-10, -10, 20, 10, 10, 60);
}

/**
 * RTB - Return To Base: returns leader to base 
 */
void UAVSimulator::RTB() {
	pathfinder.plan(swarm[0].get_pos(), {0.0, 0.0, 20.0});
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
UAVSimulator::UAVSimulator(int num_uavs) : 
										env(BORDER_X / RESOLUTION, BORDER_Y / RESOLUTION, BORDER_Z / RESOLUTION, RESOLUTION),
										pathfinder(env)
{
	// create base UAVs at a common starting point and base altitude
	swarm.reserve(num_uavs); // allocates memory to reduce resizing slowdowns
	for (int i = 0; i < num_uavs; i++)
	{
		// leader and followers start co-located; formation offsets will spread them out
		swarm.push_back(UAV(i, 8000 + i, 0.0, 0.0, 20.0, env));
		// give everyone an initial forward velocity along +Y
		swarm[i].set_velocity(0.0, 1.0, 0.0); // cruisin on y axis
	}

	// set initial formation (LINE as default) and compute offsets
	change_formation(LINE);

	// apply rotated formation offsets around the leader so the swarm starts in formation
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

		// leader position and velocity define the frame for rotation
		double leader_x = swarm[leader_idx].get_x();
		double leader_y = swarm[leader_idx].get_y();
		double leader_z = swarm[leader_idx].get_z();

		std::array<double, 3> leader_vel = {
			swarm[leader_idx].get_velx(),
			swarm[leader_idx].get_vely(),
			swarm[leader_idx].get_velz()};

		// if leader is stationary, assume a default forward heading along +Y
		double speed = std::sqrt(leader_vel[0] * leader_vel[0] + leader_vel[1] * leader_vel[1]);
		if (speed < 1e-3)
		{
			leader_vel = {0.0, 1.0, 0.0};
		}

		SwarmCoordinator &coords = swarm[leader_idx].get_SwarmCoord();

		for (int i = 0; i < num_uavs; i++)
		{
			std::array<double, 3> base_offset = coords.get_formation_offset(i);
			std::array<double, 3> rotated_offset = coords.rotate_offset_3d(base_offset, leader_vel);

			swarm[i].set_position(
				leader_x + rotated_offset[0],
				leader_y + rotated_offset[1],
				leader_z + rotated_offset[2]);
		}
	}

	std::cout << "Created swarm with " << num_uavs << " UAVs" << std::endl;
	print_swarm_status();

	// Set Up Environment
	env.generate_random_obstacles(40);
	// generate_test_obstacles(); 					// for testing
	env.environment_to_rust(RUST_UDP_PORT);

	std::array<double, 3> startXYZ = swarm[0].get_pos();
	std::array<double, 3> goalXYZ  = {0, 200, 300};		// make argv?
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

	running = true;

	// start_turn_timer();

	std::thread([this]()
				{
		using namespace std::chrono;
		const auto sleep_duration = milliseconds(int(1000 * UAVDT));    // 20 Hz Updates with .05 UAVDT
		const int telemetry_port = 6000;

		while (running) {
			for (auto &uav : swarm) {
				if (uav.get_id() == 0) // comment out these two lines if not functioning
					pathfollower->update_leader_velocity(UAVDT);
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

/**
 * create_formation_random - randomizes the placement of UAVs
 * @num_uavs: number of uavs to generate
 */
void UAVSimulator::create_formation_random(int num_uavs)
{
	int i;
	const double base_altitude = 50.00;
	std::random_device rd;	/* randomizer */
	std::mt19937 gen(rd()); /* super randomizer */

	// generator of a super random number within a bounded uniform distribution
	std::uniform_real_distribution<> distrib(-10.0, 10.0);

	// generate a swarm of uavs
	for (i = 0; i < num_uavs; i++)
	{
		double x, y, z;

		if (i == 0)
		{
			// place leader at origin at base altitude
			x = 0.0;
			y = 0.0;
			z = base_altitude;
		}
		else
		{
			// randomly place other uavs around leader
			x = distrib(gen);
			y = distrib(gen);
			z = distrib(gen) + base_altitude;
		}

		int uav_port = 8000 + i;
		UAV uav(i, uav_port, x, y, z);

		// set initial velocity to move forward in negative Y direction
		const double forward_speed = -1.0;
		(void)forward_speed;			 // currently unused
		uav.set_velocity(0.0, 0.0, 0.0); // 0.0, forward_speed, 0.0);

		swarm.push_back(uav);
	}
}

/**
 * create_formation_line - create and places UAVs in a line
 * @num_uavs: number of uavs to generate
 */
void UAVSimulator::create_formation_line(int num_uavs)
{
	int i;
	const double spacing = 10.0; // might make dependent on user-input
	const double base_altitude = 50.0;

	for (i = 0; i < num_uavs; i++)
	{
		double x, y, z;

		// leader front and center
		if (i == 0)
		{
			x = 0.0;
			y = 0.0;
		}
		else
		{
			int wing = (i + 1) / 2;			  // 1,1,2,2,...
			int side = (i % 2 == 1) ? -1 : 1; // left/right

			x = side * wing * spacing; // horizontal offset
			y = 0;					   // distance behind leader
		}

		z = base_altitude;

		int uav_port = 8000 + i;
		UAV uav(i, uav_port, x, y, z);
		swarm.push_back(uav);
	}
}

/**
 * create_formation_vee - creates and places uavs in a flying vee
 * @num_uavs: number of uavs to generate
 */
void UAVSimulator::create_formation_vee(int num_uavs)
{
	int i;
	const double spacing = 10.0; // might make dependent on user-input
	const double base_altitude = 50.0;

	for (i = 0; i < num_uavs; i++)
	{
		double x, y, z;

		// leader front and center
		if (i == 0)
		{
			x = 0.0;
			y = 0.0;
		}
		else
		{
			// VEE formation - two per wing
			int wing = (i + 1) / 2;			  // 1,1,2,2,...
			int side = (i % 2 == 1) ? -1 : 1; // left/right

			x = side * wing * spacing; // horizontal offset
			y = -wing * spacing;	   // distance behind leader (flipped)
		}

		z = base_altitude;

		int uav_port = 8000 + i;
		UAV uav(i, uav_port, x, y, z);
		swarm.push_back(uav);
	}
}

/**
 * create_formation_circle - creates and places uavs in a circle
 * @num_uavs: number of uavs to generate
 */
void UAVSimulator::create_formation_circle(int num_uavs)
{
	int i;
	const double radius = 10.0; // might make dependent on user-input
	const double base_altitude = 50.0;

	for (i = 0; i < num_uavs; i++)
	{
		double x, y, z;

		// leader front and center
		if (i == 0)
		{
			x = 0.0;
			y = 0.0;
		}
		else
		{
			double angle = 2.0 * M_PI * i / num_uavs;
			x = radius * std::cos(angle);
			y = radius * std::sin(angle);
		}

		z = base_altitude;

		int uav_port = 8000 + i;
		UAV uav(i, uav_port, x, y, z);
		swarm.push_back(uav);
	}
}

/**
 * set_formation_line - [DEPRECATED] sets UAVs in a line based on current leader
 * @num_uavs: number of uavs to position
 */
void UAVSimulator::set_formation_line(int num_uavs)
{
	int i;
	const double spacing = 10.0; // might make dependent on user-input
	double leader_y = swarm[0].get_y();
	double leader_z = swarm[0].get_z();

	for (i = 1; i < num_uavs; i++)
	{
		double x;

		int wing = (i + 1) / 2;			  // 1,1,2,2,...
		int side = (i % 2 == 1) ? -1 : 1; // left/right

		x = side * wing * spacing; // horizontal offset

		swarm[i].set_position(x, leader_y, leader_z);
	}
}

/**
 * set_formation_vee - [DEPRECATED] sets UAVs in a V based on current leader
 * @num_uavs: number of uavs to position
 */
void UAVSimulator::set_formation_vee(int num_uavs)
{
	int i;
	const double spacing = 10.0; // might make dependent on user-input
	double leader_x = swarm[0].get_x();
	double leader_y = swarm[0].get_y();
	double leader_z = swarm[0].get_z();

	for (i = 1; i < num_uavs; i++)
	{
		double x, y;

		// VEE formation - two per wing
		int wing = (i + 1) / 2;			  // 1,1,2,2,...
		int side = (i % 2 == 1) ? -1 : 1; // left/right

		x = leader_x + side * wing * spacing; // horizontal offset
		y = leader_y - wing * spacing;		  // distance behind leader (flipped)

		swarm[i].set_position(x, y, leader_z);
	}
}

/**
 * set_formation_circle - [DEPRECATED] sets UAVs in a circle based on current leader
 * @num_uavs: number of uavs to position
 */
void UAVSimulator::set_formation_circle(int num_uavs)
{
	int i;
	const double radius = 10.0; // might make dependent on user-input
	double leader_x = swarm[0].get_x();
	double leader_y = swarm[0].get_y();
	double leader_z = swarm[0].get_z();

	for (i = 1; i < num_uavs; i++)
	{
		double x, y;

		double angle = 2.0 * M_PI * i / num_uavs;
		x = leader_x + radius * std::cos(angle);
		y = leader_y + radius * std::sin(angle);

		swarm[i].set_position(x, y, leader_z);
	}
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

			if (swarm.empty())
				continue;

			// make leader's ID 0 instead of assuming UAV at index 0 is leader
			size_t leader_idx = 0;
			for (size_t i = 0; i < swarm.size(); i++)
			{
				if (swarm[i].get_id() == 0)
				{
					leader_idx = i;
					break;
				}
			}

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

			// update leader altitude
			double x = swarm[leader_idx].get_x();
			double y = swarm[leader_idx].get_y();
			double z = swarm[leader_idx].get_z() + delta;
			swarm[leader_idx].set_position(x, y, z);
		}

		// clear buffer for next recvfrom
		memset(buffer, 0, sizeof(buffer));
	}
}
