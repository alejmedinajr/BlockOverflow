import heapq
from collections import deque
import numpy as np

class SearchAlgorithms:
    """
    A collection of search algorithms optimized with lambda functions and list comprehensions.
    Each function returns the path taken, which can be used for redstone block placement in Minecraft.
    """

    def __init__(self):
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up

    def _reconstruct_path(self, came_from, start, goal):
        """Helper function to reconstruct the path from start to goal."""
        path, current = [], goal
        while current in came_from:
            path.append(current)  # Keep (x, y) order consistent
            current = came_from[current]
        path.append(start)
        path.reverse()  # Ensure correct order from start to goal
        return path if path else []

    def _is_valid(self, maze, position):
        """Check if a position is within the maze bounds and walkable."""
        x, y = position  # Keep order consistent
        return 0 <= y < len(maze) and 0 <= x < len(maze[0]) and maze[y][x] in [0, 2, 3]

    def breadth_first_search(self, start, goal, maze):
        """Breadth-First Search (BFS) using a queue."""
        queue, visited, came_from = deque([start]), {start}, {}
        visited_nodes = []

        while queue:
            node = queue.popleft()

            if node == (goal[1], goal[0]):
                final_path = self._reconstruct_path(came_from, (start[1],start[0]), (goal[1], goal[0]))
                return visited_nodes, final_path  # Stop immediately

            if self._is_valid(maze, node): visited_nodes.append(node)

            for dx, dy in self.directions:
                neighbor = (node[0] + dx, node[1] + dy)
                if neighbor not in visited and self._is_valid(maze, neighbor):
                    queue.append(neighbor)
                    visited.add(neighbor)
                    came_from[neighbor] = node

        return visited_nodes, []  # Return empty path if goal not found

    def depth_first_search(self, start, goal, maze):
        """Depth-First Search (DFS) using a stack."""
        stack, visited, came_from = [start], {start}, {}
        visited_nodes = []

        while stack:
            node = stack.pop()

            if node == (goal[1], goal[0]):
                final_path = self._reconstruct_path(came_from, (start[1],start[0]), (goal[1], goal[0]))
                return visited_nodes, final_path  # Stop immediately

            if self._is_valid(maze, node): visited_nodes.append(node)

            for dx, dy in self.directions:
                neighbor = (node[0] + dx, node[1] + dy)
                if neighbor not in visited and self._is_valid(maze, neighbor):
                    stack.append(neighbor)
                    visited.add(neighbor)
                    came_from[neighbor] = node

        return visited_nodes, []  # Return empty path if goal not found

    def greedy_search(self, start, goal, maze, use_g_score=True):
        """Greedy search that can operate as either Greedy Best-First Search (ignoring g_score) or A* (including g_score)."""
        heuristic = lambda p: abs(p[0] - goal[0]) + abs(p[1] - goal[1])
        open_set, came_from, g_score = [(0, start)], {}, {start: 0}
        visited_nodes = []

        while open_set:
            _, node = heapq.heappop(open_set)

            if node == (goal[1], goal[0]):
                final_path = self._reconstruct_path(came_from, (start[1],start[0]), (goal[1], goal[0]))
                return visited_nodes, final_path

            if self._is_valid(maze, node): visited_nodes.append(node)

            for dx, dy in self.directions:
                neighbor = (node[0] + dx, node[1] + dy)
                if neighbor not in g_score and self._is_valid(maze, neighbor):
                    movement_cost = 1 + (abs(neighbor[0] - start[0]) + abs(
                        neighbor[1] - start[1])) * 0.05  # Dynamic g-score based on distance
                    g_score[neighbor] = g_score[node] + movement_cost if use_g_score else 0
                    priority = heuristic(neighbor) if not use_g_score else g_score[neighbor] + heuristic(neighbor)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = node

        return visited_nodes, []  # Return empty path if goal not found

    def jump_point_search(self, start, goal, maze):
        """Jump Point Search (JPS) for optimal pathfinding in grid-based mazes."""

        def heuristic(p): return abs(p[0] - goal[0]) + abs(p[1] - goal[1])

        open_set, came_from, g_score = [(0, start)], {}, {start: 0}
        visited_nodes = []

        while open_set:
            _, node = heapq.heappop(open_set)

            if node == (goal[1], goal[0]):
                final_path = self._reconstruct_path(came_from, (start[1], start[0]), (goal[1], goal[0]))
                return visited_nodes, final_path

            if self._is_valid(maze, node):
                visited_nodes.append(node)

            for dx, dy in self.directions:
                jump_point = self._jump(node, dx, dy, (goal[1], goal[0]), maze)
                if jump_point:
                    cost = g_score[node] + 1
                    if jump_point not in g_score or cost < g_score[jump_point]:
                        g_score[jump_point] = cost
                        priority = cost + heuristic(jump_point)
                        heapq.heappush(open_set, (priority, jump_point))
                        came_from[jump_point] = node

        return visited_nodes, []  # Return empty path if goal not found

    def _jump(self, current, dx, dy, goal, maze):
        """Helper function to jump along a given direction in Jump Point Search."""
        x, y = current[0] + dx, current[1] + dy
        if not self._is_valid(maze, (x, y)):
            return None
        if (x, y) == goal:
            return (x, y)

        if (dx != 0 and ((self._is_valid(maze, (x, y - 1)) and not self._is_valid(maze, (x - dx, y - 1))) or
                         (self._is_valid(maze, (x, y + 1)) and not self._is_valid(maze, (x - dx, y + 1))))) or \
                (dy != 0 and ((self._is_valid(maze, (x - 1, y)) and not self._is_valid(maze, (x - 1, y - dy))) or
                              (self._is_valid(maze, (x + 1, y)) and not self._is_valid(maze, (x + 1, y - dy))))):
            return (x, y)

        return self._jump((x, y), dx, dy, goal, maze)