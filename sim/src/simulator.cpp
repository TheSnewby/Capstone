#include "simulator.h"
#include "uav.h"

/**
 * print_swarm_status: prints all UAV's position and velocity to stdout
 */
void UAVSimulator::print_swarm_status() {
	size_t swarm_size = swarm.size();

	std::cout << std::fixed << std::setprecision(2);
	std::cout << "\nPrinting current swarm.\n" << std::endl;
	std::cout <<"ID: Position X, Y, Z. Velocity: vx, vy, vz" << std::endl;

	for (size_t i = 0; i < swarm_size; i++)
	{
		std::cout << swarm[i].get_id() << ": Position " << swarm[i].get_x() <<
		", " << swarm[i].get_y() << ", " << swarm[i].get_z() << ". Velocity: " <<
		swarm[i].get_velx() << ", " << swarm[i].get_vely() << ", " << swarm[i].get_velz() << std::endl; 
	}
};

/**
 * Constructor for UAVSimulator
 */
UAVSimulator::UAVSimulator(int num_uavs) {
	swarm.reserve(num_uavs); 				// allocates memory to reduce resizing slowdowns
	create_formation_random(num_uavs);		// default, but can change to anything
	// create_formation_circle(num_uavs); 	// for testing each formation creator
	// create_formation_line(num_uavs);
	// create_formation_vee(num_uavs);

	std::cout << "Created swarm with " << num_uavs << " UAVs" << std::endl;
	print_swarm_status();
	// set_formation_line(num_uavs);		// for testing each formation setter
	// print_swarm_status();
	// set_formation_vee(num_uavs);
	// print_swarm_status();
	// set_formation_circle(num_uavs);
	// print_swarm_status();
};

/**
 * ~UAVSimulator - Destructor for UAVSimulator
 */
UAVSimulator::~UAVSimulator() {
	stop_sim();
}

/**
 * start_sim -	starts the simulation loop in a separate thread,
 *				updating UAV positions and sending telemetry to server
 */
void UAVSimulator::start_sim() {
	if (running)
		return;

	running = true;

	std::thread([this]() {
		using namespace std::chrono;
		const auto sleep_duration = milliseconds(50);    // 20 updates per second
		const int telemetry_port = 6000;

		while (running) {
			for (auto &uav : swarm) {
				uav.update_position(UAVDT); // found in uav.h
				uav.uav_to_telemetry_server(telemetry_port);
			}

			std::this_thread::sleep_for(sleep_duration);
		}
	}).detach();
}

void UAVSimulator::stop_sim() {
	running = false;

}

/**
 * change_formation - changes formation of swarm
 * @f: formation enum (1: LINE, 2: FLYING_V, 3: CIRCLE)
 */
void UAVSimulator::change_formation(formation f) {
	int uav_nums = swarm.size();

	if (f == 1)
	{
		set_formation_line(uav_nums);
		std::cout << "Formation changed to LINE." << std::endl;
	} else if (f == 2)
	{
		set_formation_vee(uav_nums);
		std::cout << "Formation changed to FLYING VEE." << std::endl;
	} else if (f == 3)
	{
		set_formation_circle(uav_nums);
		std::cout << "Formation changed to CIRCLE." << std::endl;
	}
}

/**
 * create_formation_random - randomizes the placement of UAVs
 * @num_uavs: number of uavs to generate
 */
void UAVSimulator::create_formation_random(int num_uavs) {
	int i;
	const double base_altitude = 50.00;
	std::random_device rd; /* randomizer */
	std::mt19937 gen(rd()); /* super randomizer */

	// generator of a super random number within a bounded uniform distribution
	std::uniform_real_distribution<> distrib(-10.0, 10.0);

	// generate a swarm of uavs
	for (i = 0; i < num_uavs; i++) {
		if (i == 0)
			swarm.push_back(UAV(i, 8000 + i, 0, 0, base_altitude));
		else
			swarm.push_back(UAV(i, 8000 + i, distrib(gen), distrib(gen), distrib(gen) + base_altitude));
	}
}

/**
 * create_formation_line - create and places UAVs in a line
 * @num_uavs: number of uavs to generate
 */
void UAVSimulator::create_formation_line(int num_uavs) {
	int i;
	const double spacing = 10.0; //might make dependent on user-input
	const double base_altitude = 50.0;

	for (i = 0; i < num_uavs; i++) {
		double x, y, z;

		// leader front and center
		if (i == 0) {
			x = 0.0;
			y = 0.0;
		} else {
			int wing = (i + 1) / 2;              // 1,1,2,2,...
			int side = (i % 2 == 1) ? -1 : 1;    // left/right

			x = side * wing * spacing;          // horizontal offset
			y = 0;                 				// distance behind leader
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
void UAVSimulator::create_formation_vee(int num_uavs) {
	int i;
	const double spacing = 10.0; //might make dependent on user-input
	const double base_altitude = 50.0;

	for (i = 0; i < num_uavs; i++) {
		double x, y, z;

		// leader front and center
		if (i == 0) {
			x = 0.0;
			y = 0.0;
		} else {
			// VEE formation - two per wing
			int wing = (i + 1) / 2;              // 1,1,2,2,...
			int side = (i % 2 == 1) ? -1 : 1;    // left/right

			x = side * wing * spacing;          // horizontal offset
			y = wing * spacing;                 // distance behind leader
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
void UAVSimulator::create_formation_circle(int num_uavs) {
	int i;
	const double radius = 10.0; //might make dependent on user-input
	const double base_altitude = 50.0;

	for (i = 0; i < num_uavs; i++) {
		double x, y, z;

		// leader front and center
		if (i == 0) {
			x = 0.0;
			y = 0.0;
		} else {
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
 * set_formation_line - randomizes the placement of UAVs
 * @num_uavs: number of uavs to generate
 */
void UAVSimulator::set_formation_line(int num_uavs) {
	int i;
	const double spacing = 10.0; //might make dependent on user-input
	double leader_y = swarm[0].get_y();
	double leader_z = swarm[0].get_z();

	for (i = 1; i < num_uavs; i++) {
		double x, y, z;

		int wing = (i + 1) / 2;              // 1,1,2,2,...
		int side = (i % 2 == 1) ? -1 : 1;    // left/right

		x = side * wing * spacing;          // horizontal offset

		swarm[i].set_position(x, leader_y, leader_z);
	}
}


/**
 * set_formation_vee - randomizes the placement of UAVs
 * @num_uavs: number of uavs to generate
 */
void UAVSimulator::set_formation_vee(int num_uavs) {
	int i;
	const double spacing = 10.0; //might make dependent on user-input
	double leader_x = swarm[0].get_x();
	double leader_y = swarm[0].get_y();
	double leader_z = swarm[0].get_z();

	for (i = 1; i < num_uavs; i++) {
		double x, y, z;

		// VEE formation - two per wing
		int wing = (i + 1) / 2;              // 1,1,2,2,...
		int side = (i % 2 == 1) ? -1 : 1;    // left/right

		x = leader_x + side * wing * spacing;          // horizontal offset
		y = leader_y + wing * spacing;                 // distance behind leader

		z = leader_z;

		swarm[i].set_position(x, y, z);
	}
}

/**
 * set_formation_circle - randomizes the placement of UAVs
 * @num_uavs: number of uavs to generate
 */
void UAVSimulator::set_formation_circle(int num_uavs) {
	int i;
	const double radius = 10.0; //might make dependent on user-input
	double leader_x = swarm[0].get_x();
	double leader_y = swarm[0].get_y();
	double leader_z = swarm[0].get_z();

	for (i = 1; i < num_uavs; i++) {
		double x, y, z;

		double angle = 2.0 * M_PI * i / num_uavs;
		x = leader_x + radius * std::cos(angle);
		y = leader_y + radius * std::sin(angle);

		z = leader_z;

		swarm[i].set_position(x, y, z);
	}
}
