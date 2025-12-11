#pragma once

#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <random>
#include <cerrno>
#include <sstream>

/**
 * Environment Class Concepts
 *
 * World space: (x,y,z) in meters, continuous, origin=(0,0,0)
 * Grid space: (i,j,k) in integer cell indices, origin cell (0,0,0)
 * 	maps to world coordinate (world_min_x, world_min_y, world_min_z)
 *
 * grid_i = floor((x − world_min_x)/resolution)
 * world_x = world_min_x + (i + 0.5)*resolution
 */

class Environment
{
private:
	int nx, ny, nz;					// number of cells in X, Y, Z
	double resolution;				// meters per cell
	std::array<double, 3> origin;	// world coordinates for (0, 0, 0)
	std::vector<uint8_t> occupancy; // flat array: 0 - free, 1 - blocked
	nlohmann::json msg;				// json to send to telemetry (to send to rust)
	bool goal_set = false;
	std::array<double, 4> goal_data{}; // x, y, z, radius

public:
	Environment(int nx_, int ny_, int nz_, double res_) : nx(nx_),
														  ny(ny_),
														  nz(nz_),
														  resolution(res_),
														  origin({-nx * res_ / 2.0, -ny * res_ / 2.0, 0.0}),
														  occupancy(nx_ * ny_ * nz_, 0)
	{
		msg["type"] = "environment";
		msg["obstacles"] = nlohmann::json::array();
		msg["goal"] = nullptr;
	}

	// Getters
	int getNx() const { return nx; }
	int getNy() const { return ny; }
	int getNz() const { return nz; }
	double getResolution() const { return resolution; }
	std::array<double, 3> getOrigin() const { return origin; }

	bool inBounds(int i, int j, int k) const;
	void setBlock(int i, int j, int k, bool blocked);
	bool isBlocked(int i, int j, int k) const;

	std::array<int, 3> toGrid(const std::array<double, 3> &point) const;
	std::array<double, 3> toWorld(int i, int j, int k) const;

	void addBox(double x0, double y0, double z0, double x1, double y1, double z1);
	void addSphere(const std::array<double, 3> &center, double radius);
	void addCylinder(const std::array<double, 3> &center, double radius, double height);
	void generate_random_obstacles(int count);
	void setGoal(const std::array<double, 3>& center, double radius);
	int environment_to_rust(int port);

private:
	inline int idx(int i, int j, int k) const { return ((k * ny + j) * nx + i); }
};
