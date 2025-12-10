#include "environment.h"

using json = nlohmann::json;

struct Cylinder
{
	double x, y, z, radius, height;

	Cylinder(double x_,double y_,double z_,
           double rad_,double h_)
    : x(x_),y(y_),z(z_),radius(rad_),height(h_) {}
};
struct Box
{
	double x, y, z, width, depth, height;

	Box(double x_,double y_,double z_, double wid_, double dep_,double h_)
    : x(x_),y(y_),z(z_), width(wid_), depth(dep_),height(h_) {}
};
struct Sphere
{
	double x, y, z, radius;

	Sphere(double x_,double y_,double z_, double rad_)
    : x(x_),y(y_),z(z_),radius(rad_) {}
};

void to_json(json &j, Cylinder const &c)
{
	j = {
		{"type", "cylinder"},
		{"x", c.x},
		{"y", c.y},
		{"z", c.z},
		{"radius", c.radius},
		{"height", c.height}};
}

void to_json(json &j, Box const &b)
{
	j = {
		{"type", "box"},
		{"x", b.x},
		{"y", b.y},
		{"z", b.z},
		{"width", b.width},
		{"depth", b.depth},
		{"height", b.height}};
}

void to_json(json &j, Sphere const &s)
{
	j = {
		{"type", "sphere"},
		{"x", s.x},
		{"y", s.y},
		{"z", s.z},
		{"radius", s.radius}};
}

/**
 * inBounds - ensures location within world bounds
 * @i: x-value
 * @j: y-value
 * @k: z-value
 *
 * Return: 1 if in bounds, 0 otherwise
 */
bool Environment::inBounds(int i, int j, int k) const
{
	return (i >= 0 && i < nx && j >= 0 && j < ny && k >= 0 && k < nz);
};

/**
 * SetBlock - marks a location as blocked
 * @i: x-value
 * @j: y-value
 * @k: z-value
 * @blocked: 1 if blocked, 0 otherwise
 */
void Environment::setBlock(int i, int j, int k, bool blocked)
{
	occupancy[idx(i, j, k)] = blocked;
}

/**
 * isBlocked - checks if a location is blocked
 * @i: x-value
 * @j: y-value
 * @k: z-value
 *
 * Return: 1 if in bounds, 0 otherwise
 */
bool Environment::isBlocked(int i, int j, int k) const
{
	if (inBounds(i, j, k))
		return (occupancy[idx(i, j, k)]);
	else
		return true;
}

/**
 * toGrid - converts 3d coord from world space to grid space
 * @point: 3d coord in world space
 *
 * Return: 3d coords in grid space
 */
std::array<int, 3> Environment::toGrid(const std::array<double, 3> &point) const
{
	double x = (point[0] - origin[0]) / resolution;
	double y = (point[1] - origin[1]) / resolution;
	double z = (point[2] - origin[2]) / resolution;

	int x_int = int(floor(x));
	int y_int = int(floor(y));
	int z_int = int(floor(z));

	return {x_int, y_int, z_int};
}

/**
 * toWorld - converts 3d coord from grid space to world space
 * @point: 3d coord in grid space
 *
 * Return: 3d coords in world space
 */
std::array<double, 3> Environment::toWorld(int i, int j, int k) const
{
	double x = origin[0] + (i + 0.5) * resolution;
	double y = origin[1] + (j + 0.5) * resolution;
	double z = origin[2] + (k + 0.5) * resolution;

	return {x, y, z};
}

/**
 * addBox - creates a box in grid space and sets it as blocked using world space coords
 * @x0: initial x corner
 * @y0: initial y corner
 * @z0: initial z corner
 * @x1: final x corner
 * @y1: final y corner
 * @z1: final z corner
 */
void Environment::addBox(double x0, double y0, double z0, double x1, double y1, double z1)
{
	std::array<int, 3> g0 = toGrid({x0, y0, z0});
	std::array<int, 3> g1 = toGrid({x1, y1, z1});
	int safety_margin = 1; // 1 or 2 grids. adds a phantom boundary to base a repulsion force from

	// place in valid index range
	int i0 = std::max(0, std::min(nx - 1, std::min(g0[0], g1[0]) - safety_margin));
	int i1 = std::max(0, std::min(nx - 1, std::max(g0[0], g1[0]) + safety_margin));
	int j0 = std::max(0, std::min(ny - 1, std::min(g0[1], g1[1]) - safety_margin));
	int j1 = std::max(0, std::min(ny - 1, std::max(g0[1], g1[1]) + safety_margin));
	int k0 = std::max(0, std::min(nz - 1, std::min(g0[2], g1[2]) - safety_margin));
	int k1 = std::max(0, std::min(nz - 1, std::max(g0[2], g1[2]) + safety_margin));

	// mark
	for (int k = k0; k <= k1; k++)
		for (int j = j0; j <= j1; j++)
			for (int i = i0; i <= i1; i++)
				setBlock(i, j, k, true);

	Box b{0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1), fabs(x1 - x0), fabs(y1 - y0), fabs(z1 - z0)};
	msg["obstacles"].push_back(b);
}

/**
 * addSphere - creates a sphere in grid space and sets it as blocked using world space coords
 * @center: center of sphere
 * @radius: radius of sphere
 */
void Environment::addSphere(const std::array<double, 3> &center, double radius)
{
	std::array<int, 3> gc = toGrid(center);
	int r = int(ceil(radius / resolution));

	// iterate layers
	for (int k = gc[2] - r; k <= gc[2] + r; ++k)
	{
		if (k < 0 || k >= nz)
			continue;
		for (int j = gc[1] - r; j <= gc[1] + r; ++j)
		{
			if (j < 0 || j >= ny)
				continue;
			for (int i = gc[0] - r; i <= gc[0] + r; ++i)
			{
				if (i < 0 || i >= nx)
					continue;
				auto wc = toWorld(i, j, k);
				double dx = wc[0] - center[0];
				double dy = wc[1] - center[1];
				double dz = wc[2] - center[2];
				if (dx * dx + dy * dy + dz * dz <= radius * radius)
					setBlock(i, j, k, true);
			}
		}
	}

	Sphere s{center[0], center[1], center[2], radius};
	msg["obstacles"].push_back(s);
}

/**
 * addCylinder - adds a cylinder to grid space using world coords
 * @center: center of cylinder
 * @radius: radius of cylinder
 * @height: height of cylinder
 */
void Environment::addCylinder(const std::array<double, 3> &center, double radius, double height)
{
	std::array<int, 3> gc = toGrid(center);

	int r_cell = int(ceil(radius / resolution));
	int h_cell = int(ceil(height / 2.0));

	double r_sq = radius * radius;
	double half_h = height / 2.0;

	for (int k = gc[2] - h_cell; k <= gc[2] + h_cell; k++)
	{
		// allow k == 0 so the cylinder's base sits on the grid instead of floating
		if (k < 0 || k >= nz)
			continue;
		// world Z-value of this layer's center
		double wz = origin[2] + (k + 0.5) * resolution;
		double dz = wz - center[2];
		if (std::fabs(dz) > half_h)
			continue;

		for (int j = gc[1] - r_cell; j <= gc[1] + r_cell; j++)
		{
			if (j < 0 || j > ny)
				continue;
			// world Y-value of row's center
			double wy = origin[1] + (j + 0.5) * resolution;
			double dy = wy - center[1];

			for (int i = gc[0] - r_cell; i <= gc[0] + r_cell; i++)
			{
				if (i < 0 || i >= nx)
					continue;
				// world X-value of column's center
				double wx = origin[0] + (i + 0.5) * resolution;
				double dx = wx - center[0];

				// if x,y coords within radius and |dz| <= height / 2
				if (dx * dx + dy * dy <= r_sq)
					setBlock(i, j, k, true);
			}
		}
	}

	Cylinder c(center[0], center[1], center[2], radius, height);
	msg["obstacles"].push_back(c);
}

void Environment::generate_random_obstacles(int count)
{
	if (count <= 0)
		return;

	// reset JSON obstacle list; grid will be updated by addBox/addSphere/addCylinder
	msg["obstacles"] = json::array();

	// RNG setup for random obstacle generation
	std::mt19937 rng(std::random_device{}());

	int max_ix = std::max(0, nx - 1);
	int max_iy = std::max(0, ny - 1);

	// generate obstacles across the whole grid
	double world_min_x = origin[0];
	double world_max_x = origin[0] + (max_ix + 1) * resolution;
	double world_min_y = origin[1];
	double world_max_y = origin[1] + (max_iy + 1) * resolution;

	std::uniform_real_distribution<double> x_world_dist(world_min_x, world_max_x);
	std::uniform_real_distribution<double> y_world_dist(world_min_y, world_max_y);
	std::uniform_int_distribution<int> type_dist(0, 2); // 0=cylinder, 1=box, 2=sphere

	// size distributions in meters
	std::uniform_real_distribution<double> radius_dist(3.0, 15.0);
	std::uniform_real_distribution<double> height_dist(10.0, 60.0);
	std::uniform_real_distribution<double> box_size_dist(6.0, 20.0);

	// track placed obstacle centers and their effective radii (in XY plane)
	// c[0] = x, c[1] = y, c[2] = effective radius
	std::vector<std::array<double, 3>> placed_obstacles;
	const double spacing_buffer = 10.0; // extra clearance in meters

	// base altitude for obstacles (the grid's ground level)
	double base_z = origin[2];

	for (int n = 0; n < count; ++n)
	{
		// choose obstacle type and size first, so we know its effective footprint radius
		int t = type_dist(rng);

		double radius = 0.0;
		double width = 0.0;
		double depth = 0.0;
		double height = 0.0;
		double effective_radius = 0.0; // in XY plane

		if (t == 0)
		{
			// cylinder
			radius = radius_dist(rng);
			height = height_dist(rng);
			effective_radius = radius;
		}
		else if (t == 1)
		{
			// box
			width = box_size_dist(rng);
			depth = box_size_dist(rng);
			height = height_dist(rng);
			// approximate footprint as a circle that bounds the rectangle
			effective_radius = 0.5 * std::sqrt(width * width + depth * depth);
		}
		else
		{
			// sphere
			radius = radius_dist(rng);
			effective_radius = radius;
		}

		// now choose a random spot anywhere in the environment, with non-overlap based on effective radius
		double cx = 0.0;
		double cy = 0.0;
		bool placed = false;

		for (int attempt = 0; attempt < 20 && !placed; ++attempt)
		{
			double cand_x = x_world_dist(rng);
			double cand_y = y_world_dist(rng);

			bool too_close = false;
			for (const auto &c : placed_obstacles)
			{
				double dx = cand_x - c[0];
				double dy = cand_y - c[1];
				double min_dist = effective_radius + c[2] + spacing_buffer;
				if (dx * dx + dy * dy < min_dist * min_dist)
				{
					too_close = true;
					break;
				}
			}

			if (!too_close)
			{
				cx = cand_x;
				cy = cand_y;
				placed = true;
			}
		}

		// if we failed to find a spaced-out position in several tries, just place it anywhere
		if (!placed)
		{
			cx = x_world_dist(rng);
			cy = y_world_dist(rng);
		}

		placed_obstacles.push_back({cx, cy, effective_radius});

	if (t == 0)
	{
		// cylinder: rests on the grid
		double center_z = base_z + height / 2.0; // base at grid level
		std::array<double, 3> center{cx, cy, center_z};
		addCylinder(center, radius, height);
	}
	else if (t == 1)
	{
		// box: also on grid
		double x0 = cx - width / 2.0;
		double x1 = cx + width / 2.0;
		double y0 = cy - depth / 2.0;
		double y1 = cy + depth / 2.0;
		double z0 = 0.0;            // start at grid level
		double z1 = height;         // extend upward from ground
		cx = cx; // keep center values unchanged for JSON
		cy = cy;

		addBox(x0, y0, z0, x1, y1, z1);
	}
		else
		{
			// sphere: generated between grid level and 200m ceiling

			// keep bottom of sphere above grid level and top below 200m
			double min_center_z = base_z + radius;		   // just touching the grid
			double max_center_z = base_z + 200.0 - radius; // just below 200m

			double center_z = min_center_z;
			if (max_center_z > min_center_z)
			{
				std::uniform_real_distribution<double> center_dist(min_center_z, max_center_z);
				center_z = center_dist(rng);
			}

			std::array<double, 3> center{cx, cy, center_z};
			addSphere(center, radius);
		}
	}
}

void Environment::setGoal(const std::array<double, 3> &center, double radius)
{
	goal_set = true;
	goal_data = {center[0], center[1], center[2], radius};
	msg["goal"] = {
		{"x", center[0]},
		{"y", center[1]},
		{"z", center[2]},
		{"radius", radius},
	};
}

/**
 * environment_to_rust - sends environment json str to rust server
 */
int Environment::environment_to_rust(int port)
{
	int socketfd;
	ssize_t sendto_return = 0, json_size;
	struct sockaddr_in addr;

	std::string json_str = msg.dump();

	std::cout << "DEBUG: environment_to_rust called with string length: " << json_str.length() << std::endl;
	std::cout << "DEBUG: JSON content: '" << json_str << "'" << std::endl;

	socketfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketfd < 0)
	{
		std::cout << "DEBUG: failed to create UDP socket in json_to_rust" << std::endl;
		return 0;
	}

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
		return 0;
	}

	auto *addr_in = reinterpret_cast<sockaddr_in *>(res->ai_addr);
	addr.sin_addr = addr_in->sin_addr;
	freeaddrinfo(res);

	// send to Rust UDP listener
	sendto_return = sendto(socketfd, json_str.c_str(), json_size, 0, (struct sockaddr *)&addr, sizeof(addr));
	std::cout << "DEBUG: sendto returned " << sendto_return << " bytes" << std::endl;
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
