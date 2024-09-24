from PIL import Image
import numpy as np
from occupancy_demo import MockOccupancyGrid

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

# Example usage
pgm_file_path = 'C:\Users\ofekr\Desktop\autonomous car\code\tsp_solver\test_map.pgm'
data = load_occupancy_grid_from_pgm(pgm_file_path)

# Assuming you know the width, height, resolution, origin_x, origin_y from the map metadata or assumption
width, height = 100, 100  # Example dimensions, replace with actual values
resolution = 0.05  # Example resolution, replace with actual values
origin_x, origin_y = 0.0, 0.0  # Example origin, replace with actual values

# Create the mock occupancy grid
mock_grid = MockOccupancyGrid(width, height, resolution, origin_x, origin_y, data)
