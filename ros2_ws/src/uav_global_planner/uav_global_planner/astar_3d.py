import heapq
import math


class AStar3D:
    def __init__(self, voxel_grid):
        self.grid = voxel_grid

    def plan(self, start_world, goal_world):
        start = self.grid.world_to_grid(start_world)
        goal = self.grid.world_to_grid(goal_world)

        if self.grid.is_occupied(start):
            raise RuntimeError(f"Start position is occupied or outside grid: {start_world} -> {start}")

        if self.grid.is_occupied(goal):
            raise RuntimeError(f"Goal position is occupied or outside grid: {goal_world} -> {goal}")

        came_from = {}
        g_score = {start: 0.0}

        open_set = []
        heapq.heappush(open_set, (self._heuristic(start, goal), 0.0, start))

        visited = set()
        expanded_nodes = 0

        while open_set:
            _, current_cost, current = heapq.heappop(open_set)

            if current in visited:
                continue

            visited.add(current)
            expanded_nodes += 1

            if current == goal:
                grid_path = self._reconstruct_path(came_from, current)
                world_path = [self.grid.grid_to_world(idx) for idx in grid_path]

                return {
                    "path": world_path,
                    "grid_path": grid_path,
                    "expanded_nodes": expanded_nodes,
                    "path_length": self._path_length(world_path),
                }

            for neighbor in self._neighbors(current):
                if self.grid.is_occupied(neighbor):
                    continue

                movement_cost = self._distance(current, neighbor)
                tentative_g = current_cost + movement_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f_score = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        raise RuntimeError("A* failed: no path found")

    def _neighbors(self, index):
        ix, iy, iz = index

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue

                    yield ix + dx, iy + dy, iz + dz

    def _heuristic(self, a, b):
        return self._distance(a, b)

    def _distance(self, a, b):
        ax, ay, az = a
        bx, by, bz = b

        dx = ax - bx
        dy = ay - by
        dz = az - bz

        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _reconstruct_path(self, came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def _path_length(self, path):
        if len(path) < 2:
            return 0.0

        total = 0.0

        for i in range(1, len(path)):
            x0, y0, z0 = path[i - 1]
            x1, y1, z1 = path[i]

            dx = x1 - x0
            dy = y1 - y0
            dz = z1 - z0

            total += math.sqrt(dx * dx + dy * dy + dz * dz)

        return total
