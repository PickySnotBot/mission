import itertools

class TSPSolver:
    def __init__(self, pathfinder):
        self.pathfinder = pathfinder

    def solve(self, coordinates):
        if not coordinates:
            return []

        # Generate a distance matrix using the A* pathfinder
        distance_matrix = self._calculate_distance_matrix(coordinates)

        # Use a simple nearest neighbor heuristic for TSP
        sorted_coordinates = self._nearest_neighbor_tsp(coordinates, distance_matrix)
        
        return sorted_coordinates

    def _calculate_distance_matrix(self, coordinates):
        """Calculate the distance matrix using the A* pathfinder."""
        size = len(coordinates)
        distance_matrix = [[0] * size for _ in range(size)]

        for i, start in enumerate(coordinates):
            for j, goal in enumerate(coordinates):
                if i != j:
                    distance_matrix[i][j] = self.pathfinder.find_shortest_distance(start, goal)
        
        return distance_matrix

    def _nearest_neighbor_tsp(self, coordinates, distance_matrix):
        """Solve TSP using the nearest neighbor heuristic."""
        n = len(coordinates)
        visited = [False] * n
        path = [0]  # Start from the first point
        visited[0] = True

        for _ in range(1, n):
            last = path[-1]
            next_city = min(
                [(i, distance_matrix[last][i]) for i in range(n) if not visited[i]],
                key=lambda x: x[1]
            )[0]
            path.append(next_city)
            visited[next_city] = True

        # Convert index-based path to coordinate-based path
        sorted_coordinates = [coordinates[i] for i in path]
        return sorted_coordinates
