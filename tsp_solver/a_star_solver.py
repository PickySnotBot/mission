import heapq
import math
import numpy as np
from occupancy_demo import MockOccupancyGrid  # Import the mock class instead of OccupancyGrid

class AStarPathfinder:
    def __init__(self, occupancy_grid):
        self.occupancy_grid = occupancy_grid
        self.width = occupancy_grid.info.width
        self.height = occupancy_grid.info.height
        self.resolution = occupancy_grid.info.resolution
        self.origin_x = occupancy_grid.info.origin.position.x
        self.origin_y = occupancy_grid.info.origin.position.y
        self.grid = np.array(occupancy_grid.data).reshape((self.height, self.width))

    def heuristic(self, start, goal):
        """Heuristic function: Euclidean distance"""
        return math.sqrt((start[0] - goal[0]) ** 2 + (start[1] - goal[1]) ** 2)

    def is_within_bounds(self, point):
        """Check if a point is within the bounds of the grid"""
        x, y = point
        return 0 <= x < self.width and 0 <= y < self.height

    def is_passable(self, point):
        """Check if a point is not an obstacle (non-occupied)"""
        x, y = point
        return self.grid[y, x] == 0  # Occupied grid cells are non-zero

    def get_neighbors(self, point):
        """Get neighbors for a point in the grid"""
        neighbors = [
            (point[0] + 1, point[1]),
            (point[0] - 1, point[1]),
            (point[0], point[1] + 1),
            (point[0], point[1] - 1),
            (point[0] + 1, point[1] + 1),
            (point[0] - 1, point[1] - 1),
            (point[0] + 1, point[1] - 1),
            (point[0] - 1, point[1] + 1),
        ]
        neighbors = filter(self.is_within_bounds, neighbors)
        neighbors = filter(self.is_passable, neighbors)
        return neighbors

    def grid_to_world(self, point):
        """Convert grid coordinates to world coordinates"""
        x = point[0] * self.resolution + self.origin_x
        y = point[1] * self.resolution + self.origin_y
        return (x, y)

    def world_to_grid(self, point):
        """Convert world coordinates to grid coordinates"""
        x = int((point[0] - self.origin_x) / self.resolution)
        y = int((point[1] - self.origin_y) / self.resolution)
        return (x, y)

    def a_star(self, start, goal):
        """A* algorithm to find the shortest path between start and goal"""
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)

        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_grid:
                return g_score[current]

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return float('inf')  # Return infinity if there's no path

    def find_shortest_distance(self, start, goal):
        """Main function to be used: find the shortest distance from start to goal"""
        return self.a_star(start, goal)
