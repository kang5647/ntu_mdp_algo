import math
import numpy as np
from algo.dubins import Dubins
from consts import Direction
from algo.a_star import MazeSolver
from itertools import permutations
from algo.path_visualization import visualize_paths


def direction_to_theta(direction):
    """Convert direction enumeration to theta (angle in radians)."""
    direction_to_angle = {
        0: math.pi / 2,  # NORTH
        2: 0,  # EAST
        4: 3 * math.pi / 2,  # SOUTH
        6: math.pi,  # WEST
    }
    return direction_to_angle.get(
        direction, 0
    )  # Default to 0 radians if direction is unknown


def process_optimal_path_to_waypoints(
    obstacles_info, start_x=1, start_y=1, start_direction=Direction.EAST
):
    """
    Initializes a MazeSolver, adds obstacles, and processes the optimal path to waypoints for Dubins path calculation.

    Parameters:
    - obstacles_info: A list of dictionaries, each representing obstacle information with keys 'x', 'y', 'd', and 'id'.
    - direction_to_theta: A function that converts direction enumeration to theta (angle in radians).
    - start_x: The x-coordinate of the starting position.
    - start_y: The y-coordinate of the starting position.
    - start_direction: The starting direction, an enumeration value from Direction.

    Returns:
    - waypoints: A list of waypoints derived from the optimal path. Each waypoint is a tuple of (x, y, theta, screenshot_id).
    """
    # Initialize MazeSolver with the specified start position and direction
    maze_solver = MazeSolver(20, 20, start_x, start_y, start_direction, big_turn=None)

    # Add obstacles to the MazeSolver
    for ob in obstacles_info:
        maze_solver.add_obstacle(ob["x"], ob["y"], ob["d"], ob["id"])

    # Find the optimal path
    optimal_path, _ = maze_solver.get_optimal_order_dp(retrying=False)
    waypoints = []
    startpoint = (start_x, start_y, direction_to_theta(start_direction), -1)
    waypoints.append(startpoint)
    if optimal_path:  # Ensure there's a path found
        for point in optimal_path:
            if point.screenshot_id > 0:
                waypoint = (
                    point.x,
                    point.y,
                    direction_to_theta(point.direction),
                    point.screenshot_id,
                )
                waypoints.append(waypoint)

    return waypoints


def calculate_all_edges(waypoints, obstacles_info, obstacle_expansion):
    """
    Calculate the Dubins path between all pairs of waypoints (including start to all waypoints)
    and return a graph representation with the paths' lengths as edge weights.

    Parameters:
    - start_waypoint: The starting point.
    - waypoints: A list of waypoints.
    - local_planner: An instance of a Dubins path planner.
    - obstacles_info: Information about obstacles for collision detection.

    Returns:
    - A graph represented as a dictionary of dictionaries, where each key is a waypoint, and
      each value is a dictionary with keys as destination waypoints and values as the Dubins path lengths.
    """
    graph = {}
    local_planner = Dubins(radius=3.0, point_separation=0.5)

    # Initialize graph structure
    for waypoint in waypoints:
        graph[waypoint] = {}

    # Calculate Dubins path between all pairs of waypoints
    for i, start in enumerate(waypoints):
        for end in waypoints[i + 1 :]:
            result = local_planner.dubins_path_avoiding_obstacles(
                start, end, obstacles_info, obstacle_expansion
            )
            if result is not None:
                shortest_path_length, dubins_path, shortest_path_points = result
                graph[start][end] = (
                    shortest_path_length,
                    dubins_path,
                    shortest_path_points,
                )
                graph[end][start] = (
                    shortest_path_length,
                    dubins_path,
                    shortest_path_points,
                )  # Assuming bidirectional paths
            else:
                graph[start][end] = None
                graph[end][start] = None

    return graph


def find_shortest_path_through_all_nodes(graph, start):
    nodes = list(graph.keys())
    nodes.remove(start)

    # Initially set the shortest path details to represent an infinite path length
    shortest_path = {
        "path": None,
        "distance": float("inf"),
        "dubins_paths": [],
        "path_points": [],
    }

    # Generate all permutations of nodes to visit
    for perm in permutations(nodes):
        current_path_length = 0
        valid_path = True
        current_start = start
        dubins_paths = []
        detailed_path_points = (
            []
        )  # List to accumulate detailed path points for the current path

        # Calculate the path length for this permutation
        for node in perm:
            if graph[current_start][node] is not None:
                distance, dubins_path, points = graph[current_start][node]
                current_path_length += distance
                dubins_paths.append(dubins_path)
                detailed_path_points.extend(points[:-1])  # Avoid duplicating end points
                current_start = node
            else:
                valid_path = False
                break  # Break if there's no valid path to the next node

        # Check if this is the shortest valid path so far
        if valid_path and current_path_length < shortest_path["distance"]:
            path_representation = [str(start)] + [str(node) for node in perm]
            shortest_path["path"] = path_representation
            shortest_path["distance"] = current_path_length
            shortest_path["dubins_paths"] = dubins_paths
            shortest_path["path_points"] = detailed_path_points + [
                points[-1]
            ]  # Include the final point

        if shortest_path["path"] is not None:
            print(
                f"Shortest path visiting all nodes: {' -> '.join(map(str, shortest_path['path']))}"
            )
            print(f"Total distance: {shortest_path['distance']} units")
            return shortest_path
        else:
            print("No valid path found that visits all nodes.")
            return []


def write_commands_to_file(dubins_paths, file_path):
    with open(file_path, "w") as file:
        for path in dubins_paths:
            commands = []
            # Check for backward movement based on angle degree magnitude
            is_backward = any(
                abs(np.degrees(segment)) > 180
                for index, segment in enumerate(path)
                if index in [0, 1]
            )
            segment_commands = []
            for i, segment in enumerate(path):
                # Loop through each segment: 0 (first turn), 1 (second turn), 2 (straight)
                if i < 2:  # Turn segments
                    angle_of_turn_degrees = np.degrees(segment)
                    # Check if the turn is backward based on angle magnitude
                    if is_backward and angle_of_turn_degrees >= 0:  # Backward Left Turn
                        direction_of_turn = "BL"
                    elif (
                        is_backward and angle_of_turn_degrees < 0
                    ):  # Backward Right Turn
                        direction_of_turn = "BR"
                    else:
                        direction_of_turn = "FL" if angle_of_turn_degrees > 0 else "FR"

                    segment_commands.append(
                        f"{direction_of_turn}:{abs(angle_of_turn_degrees):.2f}"
                    )
                else:  # Straight segment
                    if segment > 0:  # Straight segment length
                        segment_commands.insert(1, f"S:{segment:.2f}")

                # If backward movement is detected, reverse the command sequence for this segment
            if is_backward:
                segment_commands = segment_commands[::-1]

                # Append the screenshot command for this segment
            segment_commands.append("SS")

            # Add the possibly reversed segment commands to the overall commands list
            commands.extend(segment_commands)

            # Write the formatted command string for this path, now correctly handling backward segments
            command_str = ", ".join(commands)
            file.write(f"[{command_str}]\n")


# obstacles_info defined as per your input
obstacles_info = [
    {"x": 11, "y": 18, "d": Direction.SOUTH, "id": 2},
    {"x": 15, "y": 11, "d": Direction.WEST, "id": 4},
    {"x": 3, "y": 17, "d": Direction.SOUTH, "id": 5},
]

# Process optimal path to waypoints using the MazeSolver
waypoints = process_optimal_path_to_waypoints(
    obstacles_info,
    start_x=1,
    start_y=1,
    start_direction=Direction.EAST,
)

print("Waypoints: ", waypoints)

# Assuming obstacle_expansion and other necessary parameters are defined
obstacle_expansion = 1.0

# Calculate all edges between waypoints using Dubins paths, considering obstacles
graph = calculate_all_edges(waypoints, obstacles_info, obstacle_expansion)

# Find the shortest path through all nodes in the graph
start_waypoint = waypoints[0]  # Assuming the first waypoint as the start
shortest_path = find_shortest_path_through_all_nodes(graph, start_waypoint)

print("Dubins path: ", shortest_path["dubins_paths"])
print(shortest_path["path_points"])
visualize_paths(shortest_path["path_points"], obstacles_info)

# Write the commands for Dubins paths to a file
commands_file_path = "dubins_commands.txt"
write_commands_to_file(shortest_path["dubins_paths"], commands_file_path)

print(f"Commands for Dubins paths have been written to {commands_file_path}.")
