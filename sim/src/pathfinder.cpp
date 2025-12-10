#include "pathfinder.h"

void Pathfinder::print_idx_path(std::vector<int> pts) {
	std::vector<std::array<double, 3>> path = flatArrayToWorldArray(pts);
	int path_size = path.size();
	std::cout << "\nPrinting the leader's A* Raw Path." << std::endl;

	for (int i = 0; i < path_size; i++) {
		std::cout << i << ": ( " << path[i][0] << ", " << path[i][1] << ", " << path[i][2] << " )"<< std::endl;
	}
};

/**
 * print_path - prints the path
 * @path: path to be printed
 */
void Pathfinder::print_xyz_path(std::vector<std::array<double, 3>> path) {
	std::cout << "\nPrinting the leader's A* Smooth Path." << std::endl;
	int path_size = path.size();

	for (int i = 0; i < path_size; i++) {
		std::cout << i << ": ( " << path[i][0] << ", " << path[i][1] << ", " << path[i][2] << " )"<< std::endl;
	}
}

/**
 * toIJK - converts flattened index back into I,J,K
 * @idx: flattened index
 */
inline std::array<int, 3> Pathfinder::toIJK(int idx) const {
	int i = idx % nx;
	int temp = idx / nx;
	int j = temp % ny;
	int k = temp / ny;
	return {i, j, k};
}

/**
 * heuristic - euclidean heuristic in space
 */
double Pathfinder::heuristic(int idx_a, int idx_b) const {
	std::array<int, 3> A = toIJK(idx_a);
	std::array<int, 3> B = toIJK(idx_b);
	double dx = A[0] - B[0];
	double dy = A[1] - B[1];
	double dz = A[2] - B[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * flatArrayToWorldArray - converts array of idx points to array of xyz
 * @flat: flat idx points array
 *
 * Return: array of world points
 */
std::vector<std::array<double, 3>> Pathfinder::flatArrayToWorldArray(const std::vector<int>& flat) {
	std::vector<std::array<double, 3>> world;
	int flat_size = flat.size();
	world.reserve(flat_size);
	for (int idx = 0; idx < flat_size; idx++) {
		std::array<int, 3> ijk = toIJK(flat[idx]);
		world.push_back(env.toWorld(ijk[0], ijk[1], ijk[2]));
	}

	return world;
}

double Pathfinder::getMoveCost(const std::array<int, 3>& move) const {
	int dx = abs(move[0]);
	int dy = abs(move[1]);
	int dz = abs(move[2]);
	int nonZeros = (dx > 0) + (dy > 0) + (dz > 0); // adds up to 1, 2, or 3

	if (nonZeros == 1)
		return (1.0);
	else if (nonZeros == 2)
		return (ROOT2);
	else
		return (ROOT3);
}

// Returns true if the straight-line segment between A and B is free of obstacles.
bool Pathfinder::isLineClear(const std::array<double, 3>& A, const std::array<double, 3>& B) const {
	double dx = B[0] - A[0];
	double dy = B[1] - A[1];
	double dz = B[2] - A[2];
	double length = std::sqrt(dx * dx + dy * dy + dz * dz);

	if (length < 1e-6)
		return true;

	// step at half a grid cell for conservative checking
	double step = res * 0.5;
	int steps = static_cast<int>(std::ceil(length / step));
	double inv_steps = (steps > 0) ? 1.0 / steps : 1.0;

	for (int s = 0; s <= steps; ++s) {
		double t = s * inv_steps;
		std::array<double, 3> p = {A[0] + t * dx, A[1] + t * dy, A[2] + t * dz};
		auto g = env.toGrid(p);
		if (!env.inBounds(g[0], g[1], g[2]) || env.isBlocked(g[0], g[1], g[2])) {
			return false;
		}
	}
	return true;
}

/**
 * findPath - finds a path from start to finish in world coords
 */
std::vector<int> Pathfinder::rawAStar (
	std::array<double, 3> worldStart,
	std::array<double, 3> worldGoal) {

	// DEBUG 
	// std::cout << "\nStarting Raw A*" << std::endl;																// DEBUG
	// std::cout << "Starting at: " << worldStart[0] <<", " << worldStart[1] <<", "<< worldStart[2] << std::endl;	// DEBUG
	// std::cout << "Ending at  : " << worldGoal[0]  <<", " << worldGoal[1]  <<", "<< worldGoal[2]  << std::endl;	// DEBUG

	// convert to grid indices
	std::array<int, 3> gs = env.toGrid(worldStart); // gs: global start in grid coords
	std::array<int, 3> gg = env.toGrid(worldGoal); 	// gg: global goal  in grid coords
	int start = toIdx(gs[0], gs[1], gs[2]);			// flattened index of start in env
	int goal  = toIdx(gg[0], gg[1], gg[2]);			// flattened index of goal  in env

	// Guard against invalid or blocked start/goal cells. If either is blocked, carve a small free bubble.
	auto ensure_free = [this](int i, int j, int k) {
		if (!env.inBounds(i, j, k))
			return false;
		// clear the cell and its immediate neighbors to avoid trapping the search
		for (int dk = -1; dk <= 1; ++dk)
			for (int dj = -1; dj <= 1; ++dj)
				for (int di = -1; di <= 1; ++di) {
					int ni = i + di, nj = j + dj, nk = k + dk;
					if (env.inBounds(ni, nj, nk))
						env.setBlock(ni, nj, nk, false);
				}
		return true;
	};

	if (!env.inBounds(gs[0], gs[1], gs[2]) || !env.inBounds(gg[0], gg[1], gg[2])) {
		std::cout << "A* failed: start or goal outside environment bounds." << std::endl;
		return {};
	}

	if (env.isBlocked(gs[0], gs[1], gs[2])) {
		ensure_free(gs[0], gs[1], gs[2]);
	}
	if (env.isBlocked(gg[0], gg[1], gg[2])) {
		ensure_free(gg[0], gg[1], gg[2]);
	}

	// DEBUG SECTION
	// std::cout << "Grid bounds check - start in bounds: " << env.inBounds(gs[0], gs[1], gs[2]) << std::endl;
	// std::cout << "Grid bounds check - goal in bounds: " << env.inBounds(gg[0], gg[1], gg[2]) << std::endl;
	// std::cout << "Grid start: (" << gs[0] << ", " << gs[1] << ", " << gs[2] << ")" << std::endl;	// DEBUG
	// std::cout << "Grid goal: (" << gg[0] << ", " << gg[1] << ", " << gg[2] << ")" << std::endl;		// DEBUG
	// std::cout << "Flat start idx: " << start << ", goal idx: " << goal << std::endl;				// DEBUG
	// std::cout << "Start blocked: " << env.isBlocked(gs[0], gs[1], gs[2]) << std::endl;
	// std::cout << "Goal blocked: " << env.isBlocked(gg[0], gg[1], gg[2]) << std::endl;

	std::array<int, 3> start_verify = toIJK(start);
	std::array<int, 3> goal_verify = toIJK(goal);
	// std::cout << "Start verification: (" << start_verify[0] << ", " << start_verify[1] << ", " << start_verify[2] << ")" << std::endl;
	// std::cout << "Goal verification: (" << goal_verify[0] << ", " << goal_verify[1] << ", " << goal_verify[2] << ")" << std::endl;

	// std::cout << "Grid dimensions: nx=" << nx << ", ny=" << ny << ", nz=" << nz << std::endl;


	int total = nx *  ny * nz; 						// total number of indices in env
	std::vector<double> gscore(total,				// stores best known g value for cell idx
		std::numeric_limits<double>::infinity());
	std::vector<int>    parent(total, -1);			// stores the previous cell's index on the best path to idx
	std::vector<bool>   closed(total, false);		// marks cells already explored (popped from the open set)

	// the open set
	std::priority_queue<Node, std::vector<Node>, NodeCmp> open; // min-heap
	gscore[start] = 0.0;
	open.push({start, heuristic(start, goal), 0.0}); // creates Node with distance to goal and an initial current cost to start location as zero

	// A* Loop
	bool reachedGoal = false;
	while (!open.empty()) {
		Node cur = open.top();					// copies top value (pop doesn't return a value)
		open.pop();								// removes the top value from open
		if (closed[cur.idx]) 					//if value is already checked, skip
			continue;
		if (cur.idx == goal) {					//endgame!
			reachedGoal = true;
			break;
		}
		closed[cur.idx] = true;

		std::array<int, 3> ijk = toIJK(cur.idx);// convert flattened index to grid space
		for (auto& nbr: nbrs) {					// iterate through all node's neighbors
			int ni = ijk[0] + nbr[0];			// index of neighbor
			int nj = ijk[1] + nbr[1];
			int nk = ijk[2] + nbr[2];

			if (!env.inBounds(ni, nj, nk))		// skip out-of-bounds locations
				continue;

			// prevent cutting through obstacle corners on diagonal moves:
			// if moving along multiple axes, require adjacent faces to be free.
			int components = (nbr[0] != 0) + (nbr[1] != 0) + (nbr[2] != 0);
			if (components >= 2) {
				if (nbr[0] != 0 && nbr[1] != 0) {
					if (env.isBlocked(ijk[0] + nbr[0], ijk[1], ijk[2]) ||
						env.isBlocked(ijk[0], ijk[1] + nbr[1], ijk[2]))
						continue;
				}
				if (nbr[0] != 0 && nbr[2] != 0) {
					if (env.isBlocked(ijk[0] + nbr[0], ijk[1], ijk[2]) ||
						env.isBlocked(ijk[0], ijk[1], ijk[2] + nbr[2]))
						continue;
				}
				if (nbr[1] != 0 && nbr[2] != 0) {
					if (env.isBlocked(ijk[0], ijk[1] + nbr[1], ijk[2]) ||
						env.isBlocked(ijk[0], ijk[1], ijk[2] + nbr[2]))
						continue;
				}
			}

			if (env.isBlocked(ni, nj, nk))		// skip blocked locations
				continue;

			int nidx = toIdx(ni, nj, nk);		// idx of n (neighbor)
			double moveCost = getMoveCost(nbr);
			double tg = gscore[cur.idx] + moveCost;	// tentative g-score. 1.0 cost: distance between cells)
			if (tg < gscore[nidx]) {			// if lower score, add to open
				gscore[nidx] = tg;				// set new score for neighbor
				parent[nidx] = cur.idx;			// set parent of current node
				double f = tg + heuristic(nidx, goal); // set A* value 
				open.push({nidx, f, tg});		// push neighbor's idx, A*, and cost of path so far 
			}
		}
	}
	// In rawAStar, after the while loop:
	if (!reachedGoal) {
		std::cout << "A* failed: open set exhausted, no path found!" << std::endl;
		return {};
	}
	std::cout << "A* succeeded: found path to goal!" << std::endl;

	// reconstruct
	std::vector<int> rev;
	for (int at = goal; at != -1; at = parent[at])	// "at" current index
		rev.push_back(at);							// build reverse path
	if (rev.back() != start) 						// no path
		return {};
	std::reverse(rev.begin(), rev.end());			// reverse from beginning to end

	print_idx_path(rev);
	return (rev);
}


/**
 * smoothPath - removes redundant waypoints in the A* path
 * @raw: raw A* path
 *
 * Return: smoothed A* path
 */
std::vector<std::array<double, 3>> Pathfinder::smoothPath(const std::vector<int>& raw){
	// copy raw path into "pts"
	std::vector<std::array<double, 3>> pts = flatArrayToWorldArray(raw);

	// Collision-aware line-of-sight simplifier: keep a waypoint only if we cannot safely
	// connect the last kept point directly to the next waypoint without hitting obstacles.
	std::vector<std::array<double, 3>> corners;
	int pts_size = pts.size();
	if (pts.empty())
		return corners;
	corners.push_back(pts.front()); // load first waypoint
	for (int i = 1; i < pts_size - 1; i++) {
		const auto& last_kept = corners.back();
		const auto& next = pts[i + 1];

		// If straight segment to the next point is blocked, keep the current waypoint.
		if (!isLineClear(last_kept, next)) {
			corners.push_back(pts[i]);
		}
	}
	corners.push_back(pts.back());					// load final waypoint
	return corners;
}


/**
 * plan - create a flight path for the leader
 * @start:  starting coords in world space
 * @goal: 	goal coords in world space
 *
 * Return: returns a full A* path for the leader
 */
std::vector<std::array<double, 3>> Pathfinder::plan(
	const std::array<double, 3>& start,
	const std::array<double, 3>& goal
) {
	std::vector<int> raw = rawAStar(start, goal);
	return (flatArrayToWorldArray(raw)); // comment out when uncommenting below
	// std::vector<std::array<double, 3>> smooth = smoothPath(raw);
	// print_xyz_path(smooth);
	// return smooth;
}
