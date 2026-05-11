from uav_global_planner.voxel_grid import VoxelGrid
from uav_global_planner.astar_3d import AStar3D


bounds = {
    "x_min": -20.0,
    "x_max": 20.0,
    "y_min": -20.0,
    "y_max": 20.0,
    "z_min": 1.0,
    "z_max": 12.0,
}

obstacles = [
    {
        "name": "building_1",
        "type": "box",
        "min": [4.0, 2.0, 0.0],
        "max": [8.0, 8.0, 10.0],
    },
    {
        "name": "tree_cluster",
        "type": "box",
        "min": [-6.0, 4.0, 0.0],
        "max": [-2.0, 10.0, 7.0],
    },
    {
        "name": "vehicle_area",
        "type": "box",
        "min": [8.0, -8.0, 0.0],
        "max": [12.0, -4.0, 3.0],
    },
]

no_fly_zones = [
    {
        "name": "restricted_zone",
        "type": "box",
        "min": [0.0, 8.0, 0.0],
        "max": [5.0, 14.0, 12.0],
    }
]

start = [0.0, 0.0, 2.0]
goal = [15.0, 12.0, 6.0]

grid = VoxelGrid(
    bounds=bounds,
    resolution=1.0,
    obstacles=obstacles,
    no_fly_zones=no_fly_zones,
    safety_radius=1.0,
)

print("Grid info:")
print(grid.describe())

planner = AStar3D(grid)
result = planner.plan(start, goal)

print()
print("A* result:")
print(f"Expanded nodes: {result['expanded_nodes']}")
print(f"Path length: {result['path_length']:.2f} m")
print(f"Number of waypoints: {len(result['path'])}")
