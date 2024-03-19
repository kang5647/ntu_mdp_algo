import matplotlib.pyplot as plt
import matplotlib.patches as patches
from consts import Direction
import numpy as np


# def visualize_paths(
#     valid_paths,
#     obstacles_info,
#     grid_size=20,
#     robot_footprint=(3, 3),
#     obstacle_footprint=(1, 1),
#     robot_info=None,
#     expansion_distance=2,
# ):
#     for index, path_info in enumerate(valid_paths):
#         fig, ax = plt.subplots(figsize=(10, 10))

#         # Set grid limits based on the specified grid size
#         ax.set_xlim(0, grid_size)
#         ax.set_ylim(0, grid_size)
#         ax.set_xticks(np.arange(0, grid_size, 1))
#         ax.set_yticks(np.arange(0, grid_size, 1))
#         ax.grid(True, which="both", linestyle="-", color="grey")
#         ax.set_aspect("equal")

#         # Plot obstacles and their expanded boundaries
#         for obstacle in obstacles_info:
#             # Original obstacle
#             ob_rect = patches.Rectangle(
#                 (obstacle["x"], obstacle["y"]),
#                 obstacle_footprint[0],
#                 obstacle_footprint[1],
#                 linewidth=1,
#                 edgecolor="black",
#                 facecolor="yellow",
#             )
#             ax.add_patch(ob_rect)

#             # Expanded boundary
#             expanded_radius = (
#                 expansion_distance + obstacle_footprint[0] / 2
#             )  # Assuming square obstacles for simplicity
#             circle = patches.Circle(
#                 (obstacle["x"] + 0.5, obstacle["y"] + 0.5),
#                 expanded_radius,
#                 color="red",
#                 fill=False,
#                 linestyle="--",
#             )
#             ax.add_patch(circle)

#             # Arrow for direction
#             dx, dy = 0, 0
#             if obstacle["d"] == Direction.NORTH:
#                 dy = 0.5
#             elif obstacle["d"] == Direction.SOUTH:
#                 dy = -0.5
#             elif obstacle["d"] == Direction.EAST:
#                 dx = 0.5
#             elif obstacle["d"] == Direction.WEST:
#                 dx = -0.5
#             ax.arrow(
#                 obstacle["x"] + 0.5,
#                 obstacle["y"] + 0.5,
#                 dx,
#                 dy,
#                 head_width=0.3,
#                 head_length=0.3,
#                 fc="black",
#                 ec="black",
#             )

#         # Plot the path
#         path_points = path_info["path_points"]
#         x_coords, y_coords = zip(*path_points)
#         ax.plot(
#             x_coords,
#             y_coords,
#             "o-",
#             markersize=5,
#             linewidth=1,
#             label=f'Path Distance: {path_info["distance"]}',
#         )

#         # Coloring the robot's start zone and indicating the direction
#         if robot_info is not None:
#             start_zone = patches.Rectangle(
#                 (robot_info["x"], robot_info["y"]),
#                 robot_footprint[0],
#                 robot_footprint[1],
#                 linewidth=2,
#                 edgecolor="r",
#                 facecolor="orange",
#                 label="Start Zone",
#             )
#             ax.add_patch(start_zone)

#             # Robot direction
#             dx, dy = 0, 0
#             if robot_info["d"] == Direction.NORTH:
#                 dy = robot_footprint[1] / 2
#             elif robot_info["d"] == Direction.SOUTH:
#                 dy = -robot_footprint[1] / 2
#             elif robot_info["d"] == Direction.EAST:
#                 dx = robot_footprint[0] / 2
#             elif robot_info["d"] == Direction.WEST:
#                 dx = -robot_footprint[0] / 2
#             ax.arrow(
#                 robot_info["x"] + robot_footprint[0] / 2,
#                 robot_info["y"] + robot_footprint[1] / 2,
#                 dx,
#                 dy,
#                 head_width=0.5,
#                 head_length=0.5,
#                 fc="blue",
#                 ec="blue",
#                 length_includes_head=True,
#             )

#         plt.legend()
#         plt.title(f"Path {index + 1} Visualization")
#         plt.show()

# Assuming Direction is defined elsewhere correctly


def visualize_paths(
    valid_paths,  # This is now expected to be a list of numpy arrays
    obstacles_info,
    grid_size=20,
    robot_footprint=(3, 3),
    obstacle_footprint=(1, 1),
    robot_info=None,
    expansion_distance=2,
):
    fig, ax = plt.subplots(figsize=(10, 10))

    # Set grid limits based on the specified grid size
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    ax.set_xticks(np.arange(0, grid_size, 1))
    ax.set_yticks(np.arange(0, grid_size, 1))
    ax.grid(True, which="both", linestyle="-", color="grey")
    ax.set_aspect("equal")

    # Plot obstacles, robot, and other elements as before...
    # (Keep previous obstacle plotting code here)

    # Adapted Plotting for path_points as a list of numpy arrays
    path_points = np.array(
        valid_paths
    )  # Convert list of numpy arrays to a 2D numpy array
    x_coords, y_coords = (
        path_points[:, 0],
        path_points[:, 1],
    )  # Unpack into x and y coordinates

    ax.plot(
        x_coords,
        y_coords,
        "o-",
        markersize=5,
        linewidth=1,
        label="Path",
    )

    # Continue with robot_info plotting and other visualization tasks...
    # (Keep previous robot plotting code here)

    plt.legend()
    plt.title("Path Visualization")
    plt.show()
