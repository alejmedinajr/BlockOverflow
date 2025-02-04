import time
import grpc
import numpy as np
import random
from minecraft_pb2 import *
import minecraft_pb2_grpc
from search_algorithms import SearchAlgorithms

# Connect to Minecraft gRPC server
channel = grpc.insecure_channel('localhost:5001')
client = minecraft_pb2_grpc.MinecraftServiceStub(channel)


def create_maze(dim):
    """Generates a random maze using depth-first search (DFS)."""
    maze = np.ones((dim * 2 + 1, dim * 2 + 1))  # Grid filled with walls

    # Start point
    x, y = (0, 0)
    maze[2 * x + 1, 2 * y + 1] = 0
    stack = [(x, y)]

    while stack:
        x, y = stack[-1]
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        random.shuffle(directions)

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < dim and 0 <= ny < dim and maze[2 * nx + 1, 2 * ny + 1] == 1:
                maze[2 * nx + 1, 2 * ny + 1] = 0
                maze[2 * x + 1 + dx, 2 * y + 1 + dy] = 0
                stack.append((nx, ny))
                break
        else:
            stack.pop()

    # Create an entrance (Start) and an exit (Goal)
    maze[1, 0] = 2  # Start position
    maze[-2, -1] = 3  # Goal position

    return maze


def clear_area(client, size, start_x=-50, start_y=4, start_z=-50):
    """Clears a space in Minecraft to place the maze."""
    client.fillCube(FillCubeRequest(
        cube=Cube(
            min=Point(x=start_x, y=start_y, z=start_z),
            max=Point(x=start_x + size, y=start_y + 5, z=start_z + size)
        ),
        type=AIR
    ))


def spawn_maze(client, maze, block_type, orientation, start_x=-25, start_y=5, start_z=-25):
    """Spawns a randomly generated maze in Minecraft with a start and end marker."""
    blocks = [
        Block(position=Point(x=start_x + x, y=start_y, z=start_z + z),
              type=block_type, orientation=orientation)
        for z in range(len(maze))
        for x in range(len(maze[0]))
        if maze[z, x] == 1
    ]

    # Add Start (Redstone Block) and End (Emerald Block)
    for z in range(len(maze)):
        for x in range(len(maze[0])):
            if maze[z, x] == 2:
                start = (z,x)
                blocks.append(Block(position=Point(x=start_x + x, y=start_y, z=start_z + z), type=REDSTONE_BLOCK,
                                    orientation=NORTH))
            elif maze[z, x] == 3:
                end = (z,x)
                blocks.append(Block(position=Point(x=start_x + x, y=start_y, z=start_z + z), type=EMERALD_BLOCK,
                                    orientation=NORTH))

    client.spawnBlocks(Blocks(blocks=blocks))
    return start, end


def visualize_search(client, path, start_x=-25, start_y=5, start_z=-25, block_type=REDSTONE_BLOCK, delay=0.05):
    """Places blocks step by step along the path to simulate a real-time search algorithm."""
    for x, z in path:
        client.spawnBlocks(Blocks(blocks=[
            Block(position=Point(x=start_x + x, y=start_y, z=start_z + z), type=block_type, orientation=NORTH)
        ]))
        time.sleep(delay)


# Generate a 50x50 maze
maze_size = 25  # Set to 25 to generate a 51x51 maze
maze = create_maze(maze_size)
print(maze)
# Clear the area before spawning
clear_area(client, size=102)

# Spawn the maze with STONE walls, Redstone start, and Emerald goal
start, goal = spawn_maze(client, maze, block_type=STONE, orientation=NORTH)

# Initialize search algorithms
search_algorithms = SearchAlgorithms()

# Run a search algorithm and visualize its path
path = search_algorithms.depth_first_search(start, goal, maze.T.tolist())
print(path)
visualize_search(client, path)

print("Randomized 50x50 maze with Start (Redstone), Goal (Emerald), and step-by-step visualization spawned in Minecraft!")

clear_area(client, size=102)
start, goal = spawn_maze(client, maze, block_type=STONE, orientation=NORTH)
path = search_algorithms.breadth_first_search(start, goal, maze.T.tolist())
print(path)
visualize_search(client, path)

print("Randomized 50x50 maze with Start (Redstone), Goal (Emerald), and step-by-step visualization spawned in Minecraft!")

clear_area(client, size=102)
start, goal = spawn_maze(client, maze, block_type=STONE, orientation=NORTH)
path = search_algorithms.uniform_cost_search(start, goal, maze.T.tolist())
print(path)
visualize_search(client, path)

print("Randomized 50x50 maze with Start (Redstone), Goal (Emerald), and step-by-step visualization spawned in Minecraft!")

clear_area(client, size=102)
start, goal = spawn_maze(client, maze, block_type=STONE, orientation=NORTH)
path = search_algorithms.a_star_search(start, goal, maze.T.tolist())
print(path)
visualize_search(client, path)

print("Randomized 50x50 maze with Start (Redstone), Goal (Emerald), and step-by-step visualization spawned in Minecraft!")
