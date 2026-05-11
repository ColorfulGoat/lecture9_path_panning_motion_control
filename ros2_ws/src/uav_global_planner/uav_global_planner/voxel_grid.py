class VoxelGrid:
    def __init__(self, bounds, resolution, obstacles=None, no_fly_zones=None, safety_radius=0.0):
        self.bounds = bounds
        self.resolution = float(resolution)
        self.obstacles = obstacles or []
        self.no_fly_zones = no_fly_zones or []
        self.safety_radius = float(safety_radius)

        self.x_min = float(bounds["x_min"])
        self.x_max = float(bounds["x_max"])
        self.y_min = float(bounds["y_min"])
        self.y_max = float(bounds["y_max"])
        self.z_min = float(bounds["z_min"])
        self.z_max = float(bounds["z_max"])

        self.size_x = int(round((self.x_max - self.x_min) / self.resolution)) + 1
        self.size_y = int(round((self.y_max - self.y_min) / self.resolution)) + 1
        self.size_z = int(round((self.z_max - self.z_min) / self.resolution)) + 1

        self.occupied = set()
        self._build_occupancy()

    def _build_occupancy(self):
        for obs in self.obstacles:
            self._mark_box_as_occupied(obs, inflate=True)

        for zone in self.no_fly_zones:
            self._mark_box_as_occupied(zone, inflate=False)

    def _mark_box_as_occupied(self, box, inflate=False):
        if box.get("type", "box") != "box":
            raise ValueError(f"Unsupported box type: {box.get('type')}")

        inflation = self.safety_radius if inflate else 0.0

        min_corner = box["min"]
        max_corner = box["max"]

        x0 = min_corner[0] - inflation
        y0 = min_corner[1] - inflation
        z0 = min_corner[2] - inflation

        x1 = max_corner[0] + inflation
        y1 = max_corner[1] + inflation
        z1 = max_corner[2] + inflation

        start = self.world_to_grid_clamped([x0, y0, z0])
        end = self.world_to_grid_clamped([x1, y1, z1])

        ix0, iy0, iz0 = start
        ix1, iy1, iz1 = end

        for ix in range(min(ix0, ix1), max(ix0, ix1) + 1):
            for iy in range(min(iy0, iy1), max(iy0, iy1) + 1):
                for iz in range(min(iz0, iz1), max(iz0, iz1) + 1):
                    self.occupied.add((ix, iy, iz))

    def world_to_grid(self, point):
        x, y, z = point

        ix = int(round((x - self.x_min) / self.resolution))
        iy = int(round((y - self.y_min) / self.resolution))
        iz = int(round((z - self.z_min) / self.resolution))

        return ix, iy, iz

    def world_to_grid_clamped(self, point):
        ix, iy, iz = self.world_to_grid(point)

        ix = max(0, min(self.size_x - 1, ix))
        iy = max(0, min(self.size_y - 1, iy))
        iz = max(0, min(self.size_z - 1, iz))

        return ix, iy, iz

    def grid_to_world(self, index):
        ix, iy, iz = index

        x = self.x_min + ix * self.resolution
        y = self.y_min + iy * self.resolution
        z = self.z_min + iz * self.resolution

        return [x, y, z]

    def is_inside_grid(self, index):
        ix, iy, iz = index

        return (
            0 <= ix < self.size_x
            and 0 <= iy < self.size_y
            and 0 <= iz < self.size_z
        )

    def is_occupied(self, index):
        if not self.is_inside_grid(index):
            return True

        return index in self.occupied

    def is_free(self, index):
        return not self.is_occupied(index)

    def describe(self):
        return {
            "size_x": self.size_x,
            "size_y": self.size_y,
            "size_z": self.size_z,
            "resolution": self.resolution,
            "occupied_cells": len(self.occupied),
        }
