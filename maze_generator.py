import time
import grpc
import numpy as np
import random
import sys
from search_algorithms import SearchAlgorithms

sys.path.append("Evocraft-py")
from minecraft_pb2 import *
import minecraft_pb2_grpc

# Connect to Minecraft gRPC server
channel = grpc.insecure_channel('localhost:5001')
client = minecraft_pb2_grpc.MinecraftServiceStub(channel)

def create_maze(dim):
    """Generates a random maze using Prim's Algorithm."""
    maze = np.ones((dim * 2 + 1, dim * 2 + 1), dtype=int)  # Grid filled with walls

    # Initialize starting point
    start_x, start_y = random.randint(0, dim - 1), random.randint(0, dim - 1)
    maze[2 * start_x + 1, 2 * start_y + 1] = 0

    # List of walls
    walls = []
    for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
        nx, ny = start_x + dx, start_y + dy
        if 0 <= nx < dim and 0 <= ny < dim: walls.append((nx, ny, start_x, start_y))  # (Wall X, Wall Y, Adjacent Path X, Adjacent Path Y)

    while walls:
        # Pick a random wall
        idx = random.randint(0, len(walls) - 1)
        x, y, px, py = walls.pop(idx)

        if maze[2 * x + 1, 2 * y + 1] == 1:  # Ensure it's still a wall
            # Convert wall into a path
            maze[2 * x + 1, 2 * y + 1] = 0
            maze[2 * px + 1 + (x - px), 2 * py + 1 + (y - py)] = 0  # Clear wall between

            # Add new walls around this cell
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < dim and 0 <= ny < dim and maze[2 * nx + 1, 2 * ny + 1] == 1:
                    walls.append((nx, ny, x, y))

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
                blocks.append(Block(position=Point(x=start_x + x, y=start_y, z=start_z + z), type=RED_GLAZED_TERRACOTTA, orientation=NORTH))
            elif maze[z, x] == 3:
                end = (z,x)
                blocks.append(Block(position=Point(x=start_x + x, y=start_y, z=start_z + z), type=RED_GLAZED_TERRACOTTA, orientation=NORTH))

    client.spawnBlocks(Blocks(blocks=blocks))
    return start, end

def visualize_search(client, visited_nodes, final_path, start_x=-25, start_y=5, start_z=-25, delay=0.01):
    """Visualizes the search process step by step, first placing visited nodes, then final path."""
    # Show visited nodes first
    for x, z in visited_nodes:
        client.spawnBlocks(Blocks(blocks=[
            Block(position=Point(x=start_x + x, y=start_y, z=start_z + z), type=LIGHT_BLUE_GLAZED_TERRACOTTA, orientation=NORTH)
        ]))
        time.sleep(delay)

    time.sleep(delay) # short delay for recording purposesp
    # Show final path in redstone blocks
    for x, z in final_path:
        client.spawnBlocks(Blocks(blocks=[
            Block(position=Point(x=start_x + x, y=start_y, z=start_z + z), type=RED_GLAZED_TERRACOTTA, orientation=NORTH)
        ]))

def generate(client, maze, maze_size=50, delay=2, algorithm="depth_first_search", **kwargs):
    """
    Generates a maze in Minecraft, clears the area, spawns the maze, and runs a specified search algorithm.

    Parameters:
    - client: Minecraft client
    - maze: The maze structure
    - maze_size: Size of the maze (default: 50)
    - delay: Delay before visualization (default: 2s)
    - algorithm: The search algorithm to use (options: "depth_first_search", "breadth_first_search", "a_star_search", "uniform_cost_search")
    """
    clear_area(client, maze_size*5)  # Clear the area before spawning
    time.sleep(delay)

    start, goal = spawn_maze(client, maze, block_type=QUARTZ_BLOCK, orientation=NORTH)  # Spawn the maze
    time.sleep(delay)

    search_algorithms = SearchAlgorithms()  # Initialize search algorithms

    # Select and run the specified search algorithm
    if hasattr(search_algorithms, algorithm):  # Ensure the algorithm exists
        search_function = getattr(search_algorithms, algorithm)
        visited, final_path = search_function(start, goal, maze.tolist(), **kwargs)
    else:  raise ValueError(f"Invalid search algorithm: {algorithm}")

    # Visualize the search process
    visualize_search(client, visited, final_path)

    print(f"Generated {maze_size}x{maze_size} maze with Start (Redstone), Goal (Emerald), and step-by-step {algorithm} visualization in Minecraft!")

maze_size, delay = 50, 2
maze = create_maze(maze_size)
time.sleep(10)
generate(client, maze, maze_size, 0, "depth_first_search")
time.sleep(10)
generate(client, maze, maze_size, 0, "breadth_first_search")
time.sleep(10)
generate(client, maze, maze_size, 0, "greedy_search", use_g_score=False)
time.sleep(10)
generate(client, maze, maze_size, 0, "greedy_search", use_g_score=True)
time.sleep(10)
generate(client, maze, maze_size, 0, "jump_point_search")