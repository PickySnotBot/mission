from PIL import Image
import numpy as np
import yaml
from occupancy_demo import MockOccupancyGrid
from a_star_solver import AStarPathfinder
from tsp_solver import TSPSolver
from plotter import Plotter

def load_occupancy_grid_from_pgm(pgm_file_path, threshold=128):
    # Open the PGM file using Pillow
    img = Image.open(pgm_file_path).convert('L')  # Convert to grayscale

    # Convert the image to a numpy array
    occupancy_grid = np.array(img)

    # Convert to occupancy grid (0 for free space, 1 for obstacles)
    occupancy_grid = (occupancy_grid < threshold).astype(int)  # Use threshold to determine obstacles

    # Optionally, flip the grid if you want the origin at the bottom-left
    occupancy_grid = np.flipud(occupancy_grid)

    return occupancy_grid.flatten()  # Flatten the grid for use in the mock grid

def load_groceries_from_yaml(yaml_file_path):
    with open(yaml_file_path, 'r') as file:
        groceries_data = yaml.safe_load(file)
    return groceries_data['groceries']

# Load occupancy grid from a .pgm file
pgm_file_path = 'C:/Users/ofekr/Desktop/autonomous car/code/tsp_solver/test_map.pgm'

# Check the actual image dimensions
img = Image.open(pgm_file_path)
width, height = img.size  # Use the actual dimensions of the image
print(f"Image dimensions: width={width}, height={height}")

# Load the occupancy grid data
data = load_occupancy_grid_from_pgm(pgm_file_path)

# Set the resolution and origin (use actual values if available)
resolution = 0.05  # Example resolution, replace with actual values
origin_x, origin_y = 0.0, 0.0  # Example origin, replace with actual values

# Create the mock occupancy grid
mock_grid = MockOccupancyGrid(width, height, resolution, origin_x, origin_y, data)

# Initialize the pathfinder
pathfinder = AStarPathfinder(mock_grid)

# Load groceries from YAML file
yaml_file_path = 'C:/Users/ofekr/Desktop/autonomous car/code/tsp_solver/groceries.yaml'
groceries = load_groceries_from_yaml(yaml_file_path)

# Extract the coordinates from the groceries
coordinates = [(item['coordinate']['x'], item['coordinate']['y']) for item in groceries]

# Initialize the TSP solver with the pathfinder
tsp_solver = TSPSolver(pathfinder)

# Solve the TSP to get the sorted list of coordinates
sorted_coordinates = tsp_solver.solve(coordinates)

print("Sorted coordinates for shortest path:")
for coord in sorted_coordinates:
    print(coord)

# Initialize the Plotter with the pathfinder
plotter = Plotter(pathfinder)

# Plot the TSP solution along with the A* paths
plotter.plot_solution(coordinates, sorted_coordinates)
