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
            path.append((current[1], current[0]))  # Swap (x, y) -> (y, x)
            current = came_from[current]
        return path[::-1] if path else []

    def _is_valid(self, maze, position):
        """Check if a position is within the maze bounds and walkable."""
        x, y = position
        return 0 <= y < len(maze) and 0 <= x < len(maze[0]) and maze[y][x] in [0, 2, 3]

    def breadth_first_search(self, start, goal, maze):
        """Breadth-First Search (BFS) using a queue."""
        queue, visited, came_from = deque([start]), {start}, {}
        while queue:
            node = queue.popleft()
            if node == goal:
                return self._reconstruct_path(came_from, start, goal)
            [queue.append(neighbor) or visited.add(neighbor) or came_from.update({neighbor: node})
             for dx, dy in self.directions if (neighbor := (node[0] + dx, node[1] + dy)) not in visited and self._is_valid(maze, neighbor)]
        return []

    def depth_first_search(self, start, goal, maze):
        """Depth-First Search (DFS) using a stack."""
        stack, visited, came_from = [start], {start}, {}
        while stack:
            node = stack.pop()
            if node == goal:
                return self._reconstruct_path(came_from, start, goal)
            [stack.append(neighbor) or visited.add(neighbor) or came_from.update({neighbor: node})
             for dx, dy in self.directions if (neighbor := (node[0] + dx, node[1] + dy)) not in visited and self._is_valid(maze, neighbor)]
        return []

    def a_star_search(self, start, goal, maze):
        """A* Search using a priority queue."""
        heuristic = lambda p: abs(p[0] - goal[0]) + abs(p[1] - goal[1])
        open_set, came_from, g_score = [(0, start)], {}, {start: 0}
        while open_set:
            _, node = heapq.heappop(open_set)
            if node == goal:
                return self._reconstruct_path(came_from, start, goal)
            [(heapq.heappush(open_set, (g_score[node] + 1 + heuristic(neighbor), neighbor)), came_from.update({neighbor: node}), g_score.update({neighbor: g_score[node] + 1}))
             for dx, dy in self.directions if (neighbor := (node[0] + dx, node[1] + dy)) not in g_score and self._is_valid(maze, neighbor)]
        return []

    def uniform_cost_search(self, start, goal, maze):
        """Uniform Cost Search (UCS) algorithm."""
        open_set, came_from, cost = [(0, start)], {}, {start: 0}
        while open_set:
            current_cost, node = heapq.heappop(open_set)
            if node == goal:
                return self._reconstruct_path(came_from, start, goal)
            [(heapq.heappush(open_set, (current_cost + 1, neighbor)), came_from.update({neighbor: node}), cost.update({neighbor: current_cost + 1}))
             for dx, dy in self.directions if (neighbor := (node[0] + dx, node[1] + dy)) not in cost and self._is_valid(maze, neighbor)]
        return []
