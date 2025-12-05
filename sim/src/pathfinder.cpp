#include "pathfinder.h"

/**
 * print_path - prints the path
 * @path: path to be printed
 */
void print_path(std::vector<std::array<double, 3>> path) {
	std::cout << "\nPrinting the leader's A* Path." << std::endl;
	int path_size = path.size();

	for (int i = 0; i < path_size; i++) {
		std::cout << "( " << path[i][0] << ", " << path[i][1] << ", " << path[i][2] << " )"<< std::endl;
	}
}

/**
 * toIJK - converts flattened index back into I,J,K
 * @idx: flattened index
 */
inline std::array<int, 3> Pathfinder::toIJK(int idx) const {
	int i = idx % nx;
	int j = (idx / nx) % ny;
	int k = idx / (nx * ny);
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
 * findPath - finds a path from start to finish in world coords
 */
std::vector<std::array<double, 3>> Pathfinder::findPath (
	std::array<double, 3> worldStart,
	std::array<double, 3> worldGoal) {

	// convert to grid indices
	std::array<int, 3> gs = env.toGrid(worldStart); // gs: global start in grid coords
	std::array<int, 3> gg = env.toGrid(worldGoal); 	// gg: global goal  in grid coords
	int start = toIdx(gs[0], gs[1], gs[2]);			// index of start in env
	int goal  = toIdx(gg[0], gg[1], gg[2]);			// index of goal  in env

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
	while (!open.empty()) {
		Node cur = open.top();					// copies top value (pop doesn't return a value)
		open.pop();								// removes the top value from open
		if (closed[cur.idx]) 					//if value is already checked, skip
			continue;
		if (cur.idx == goal) 					//endgame!
			break;
		closed[cur.idx] = true;

		std::array<int, 3> ijk = toIJK(cur.idx);// convert flattened index to grid space
		for (auto& nbr: nbrs) {					// iterate through all node's neighbors
			int ni = ijk[0] + nbr[0];			// index of neighbor
			int nj = ijk[1] + nbr[1];
			int nk = ijk[2] + nbr[2];

			if (!env.inBounds(ni, nj, nk))		// skip out-of-bounds locations
				continue;
			if (env.isBlocked(ni, nj, nk))		// skip blocked locations
				continue;

			int nidx = toIdx(ni, nj, nk);		// idx of n (neighbor)
			double tg = gscore[cur.idx] + 1.0;	// tentative g-score. 1.0 cost: distance between cells)
			if (tg < gscore[nidx]) {			// if lower score, add to open
				gscore[nidx] = tg;				// set new score for neighbor
				parent[nidx] = cur.idx;			// set parent of current node
				double f = tg + heuristic(nidx, goal); // set A* value 
				open.push({nidx, f, tg});		// push neighbor's idx, A*, and cost of path so far 
			}
		}
	}

	// reconstruct
	std::vector<int> rev;
	for (int at = goal; at != -1; at = parent[at])	// "at" current index
		rev.push_back(at);							// build reverse path
	if (rev.back() != start) 						// no path
		return {};
	std::reverse(rev.begin(), rev.end());			// reverse from beginning to end

	// convert path coords from grid to world
	std::vector<std::array<double, 3>> path;		// final path
	path.reserve(rev.size());
	for (int idx : rev) {							// all flattened indices in reversed list
		std::array<int, 3> ijk = toIJK(idx);		// convert to unflattened array of grid space
		path.push_back(env.toWorld(ijk[0], ijk[1], ijk[2])); //fill path with world space indices
	}
	print_path(path);
	return (path);
}
