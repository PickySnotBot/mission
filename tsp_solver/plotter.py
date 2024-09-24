import heapq  # Priority queue for A*
import matplotlib.pyplot as plt

class Plotter:
    def __init__(self, pathfinder):
        self.pathfinder = pathfinder

    def plot_solution(self, coordinates, sorted_coordinates):
        plt.figure(figsize=(10, 10))
        self._plot_obstacles()
        self._plot_a_star_paths(sorted_coordinates)
        self._plot_tsp_solution(coordinates, sorted_coordinates)
        self._highlight_start_end(sorted_coordinates)
        plt.title("TSP Solution with A* Paths")
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.grid(True)
        plt.show()

    def _plot_obstacles(self):
        """Plot obstacles from the occupancy grid."""
        grid = self.pathfinder.grid
        height, width = grid.shape
        for y in range(height):
            for x in range(width):
                if grid[y, x] != 0:  # Assuming non-zero values are obstacles
                    plt.scatter(x * self.pathfinder.resolution + self.pathfinder.origin_x,
                                y * self.pathfinder.resolution + self.pathfinder.origin_y,
                                color='black', s=100)

    def _plot_tsp_solution(self, coordinates, sorted_coordinates):
        """Plot the TSP solution by connecting the points in the sorted order."""
        for i in range(len(sorted_coordinates) - 1):
            start = sorted_coordinates[i]
            end = sorted_coordinates[i + 1]

            # Only plot the TSP line if there is no A* path
            path = self._get_a_star_path(start, end)
            if not path:
                plt.plot([start[0], end[0]], [start[1], end[1]], 'bo-', label="TSP Path" if i == 0 else "")

        # Plot the original points
        for coord in coordinates:
            plt.scatter(coord[0], coord[1], color='blue', s=100)

    def _plot_a_star_paths(self, sorted_coordinates):
        """Plot the A* paths between the sorted coordinates."""
        for i in range(len(sorted_coordinates) - 1):
            start = sorted_coordinates[i]
            end = sorted_coordinates[i + 1]
            path = self._get_a_star_path(start, end)

            if path:  # Check if path is not empty
                path_x, path_y = zip(*path)
                plt.plot(path_x, path_y, 'ro-', label="A* Path" if i == 0 else "")
            else:
                print(f"No valid path found between {start} and {end}.")

    def _highlight_start_end(self, sorted_coordinates):
        """Highlight the starting and ending points."""
        if sorted_coordinates:
            # Highlight the start point
            start = sorted_coordinates[0]
            plt.scatter(start[0], start[1], color='green', s=200, label="Start Point")

            # Highlight the end point
            end = sorted_coordinates[-1]
            plt.scatter(end[0], end[1], color='red', s=200, label="End Point")

    def _get_a_star_path(self, start, goal):
        """Get the full path from A*."""
        start_grid = self.pathfinder.world_to_grid(start)
        goal_grid = self.pathfinder.world_to_grid(goal)

        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.pathfinder.heuristic(start_grid, goal_grid)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_grid:
                return self._reconstruct_path(came_from, current)

            for neighbor in self.pathfinder.get_neighbors(current):
                tentative_g_score = g_score[current] + self.pathfinder.heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.pathfinder.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # Return an empty path if no path is found

    def _reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal."""
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()

        # Convert grid coordinates back to world coordinates
        return [self.pathfinder.grid_to_world(p) for p in total_path]
