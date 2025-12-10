#include "formation.h"
#include "swarm_coordinator.h"

void SwarmCoordinator::calculate_formation_offsets(int num_uavs, formation f)
{
	formation_offsets.clear();
	formation_offsets.resize(num_uavs);

	// Keep formation spacing at least the configured separation distance so boids
	// separation forces don't immediately push aircraft into a staggered layout.
	const double spacing = separation;

	switch (f)
	{
	case LINE:
		// leader at origin in local formation space
		if (num_uavs > 0)
			formation_offsets[0] = {0.0, 0.0, 0.0};

		// followers form a straight line behind the leader along -Y
		for (int i = 1; i < num_uavs; ++i)
		{
			double k = static_cast<double>(i);
			double x = 0.0;			 // no lateral offset, single column
			double y = -k * spacing; // each UAV further behind the leader
			formation_offsets[i] = {x, y, 0.0};
		}
		break;

	case FLYING_V:
		// leader at origin in local formation space
		if (num_uavs > 0)
			formation_offsets[0] = {0.0, 0.0, 0.0};

		// followers form a symmetric V trailing behind the leader.
		// We place followers in left/right pairs, each ring one step farther back.
		for (int i = 1; i < num_uavs; ++i)
		{
			int wing = (i + 1) / 2;			  // 1,1,2,2,3,3,...
			int side = (i % 2 == 1) ? -1 : 1; // -1 (left), +1 (right)
			double k = static_cast<double>(wing);

			double x = side * k * spacing; // lateral offset
			double y = -k * spacing;	   // distance behind leader (local -Y)

			formation_offsets[i] = {x, y, 0.0};
		}
		break;

	case CIRCLE:
		// Circle in local XY; altitude is controlled by the UAV's altitude controller
		// using the swarm's target_altitude, not by these local offsets.
		const double radius = spacing;

		for (int i = 0; i < num_uavs; i++)
		{
			// leader front and center
			if (i == 0)
			{
				formation_offsets[0] = {0, 0, 0};
			}
			else
			{
				double angle = 2.0 * M_PI * (i - 1) / (num_uavs - 1);
				formation_offsets[i] = {
					radius * std::cos(angle),
					radius * std::sin(angle),
					0};
			}
		}
		break;
	}
}

/**
 * rotate-offset_3d - A Rotation Matrix  on the offset to ensure the formation
 * remains orthogonal (perpendicular) to the leader's heading
 * @offset: the offsets for the formation
 * @leader_velocity: the velocity (heading and magnitude) of the leader
 *
 * Return: rotated offset array
 */
std::array<double, 3> SwarmCoordinator::rotate_offset_3d(
	const std::array<double, 3> &offset,
	const std::array<double, 3> &leader_velocity)
{
	// Handle ~zero leader velocity
	double mag = sqrt(leader_velocity[0] * leader_velocity[0] +
					  leader_velocity[1] * leader_velocity[1] +
					  leader_velocity[2] * leader_velocity[2]);
	if (mag < 1e-6)
		return offset;

	// Normalize leader velocity (forward / heading)
	std::array<double, 3> heading = {
		leader_velocity[0] / mag,
		leader_velocity[1] / mag,
		leader_velocity[2] / mag};

	// Choose a world-up vector, avoid degeneracy for near-vertical headings
	std::array<double, 3> vertical_axis = {0, 0, 1};
	if ((fabs(heading[0]) < 1e-3) && (fabs(heading[1]) < 1e-3))
		vertical_axis = {1, 0, 0};

	// Right = vertical_axis × heading  (right-handed coordinate frame)
	std::array<double, 3> right_vector = {
		vertical_axis[1] * heading[2] - vertical_axis[2] * heading[1],
		vertical_axis[2] * heading[0] - vertical_axis[0] * heading[2],
		vertical_axis[0] * heading[1] - vertical_axis[1] * heading[0]};
	double right_vector_magnitude = sqrt(right_vector[0] * right_vector[0] +
										 right_vector[1] * right_vector[1] +
										 right_vector[2] * right_vector[2]);
	if (right_vector_magnitude < 1e-6)
		right_vector_magnitude = 1e-6;
	for (int i = 0; i < 3; i++)
		right_vector[i] /= right_vector_magnitude;

	// True Vertical Axis = heading × right_vector.
	std::array<double, 3> true_vertical_axis = {
		heading[1] * right_vector[2] - heading[2] * right_vector[1],
		heading[2] * right_vector[0] - heading[0] * right_vector[2],
		heading[0] * right_vector[1] - heading[1] * right_vector[0]};
	double true_vertical_axis_magnitude = sqrt(true_vertical_axis[0] * true_vertical_axis[0] +
											   true_vertical_axis[1] * true_vertical_axis[1] +
											   true_vertical_axis[2] * true_vertical_axis[2]);
	if (true_vertical_axis_magnitude < 1e-6)
		true_vertical_axis_magnitude = 1e-6;
	for (int i = 0; i < 3; i++)
		true_vertical_axis[i] /= true_vertical_axis_magnitude;

	// Apply rotation matrix: local offset → world space
	std::array<double, 3> rotation = {
		(offset[0] * right_vector[0]) + (offset[1] * heading[0]) + (offset[2] * true_vertical_axis[0]),
		(offset[0] * right_vector[1]) + (offset[1] * heading[1]) + (offset[2] * true_vertical_axis[1]),
		(offset[0] * right_vector[2]) + (offset[1] * heading[2]) + (offset[2] * true_vertical_axis[2])};

	return rotation;
}

/**
 * get's the formation offset for this specific uav
 */
std::array<double, 3> SwarmCoordinator::get_formation_offset(int uav_id)
{
	if (uav_id >= formation_offsets.size())
	{
		std::cout << "Invalid UAV ID " << uav_id << ". size of formation_offsets: " << formation_offsets.size() << std::endl;
		return {0, 0, 0};
	}
	return (formation_offsets[uav_id]);
}

// add listeners for sliders
