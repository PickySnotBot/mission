import tkinter as tk
from tkinter import messagebox
import yaml
from PIL import Image
import numpy as np
from occupancy_demo import MockOccupancyGrid
from a_star_solver import AStarPathfinder
from tsp_solver import TSPSolver
from plotter import Plotter

def load_occupancy_grid_from_pgm(pgm_file_path, threshold=128):
    img = Image.open(pgm_file_path).convert('L')  # Convert to grayscale
    occupancy_grid = np.array(img)
    occupancy_grid = (occupancy_grid < threshold).astype(int)  # Use threshold to determine obstacles
    occupancy_grid = np.flipud(occupancy_grid)  # Flip the grid if needed
    return occupancy_grid.flatten()  # Flatten the grid for use in the mock grid

def load_groceries_from_yaml(yaml_file_path):
    with open(yaml_file_path, 'r') as file:
        groceries_data = yaml.safe_load(file)
    return groceries_data['groceries']

def run_algorithm(selected_groceries, pgm_file_path):
    if not selected_groceries:
        messagebox.showwarning("No Groceries Selected", "Please select at least one grocery.")
        return

    # Load the occupancy grid
    img = Image.open(pgm_file_path)
    width, height = img.size
    data = load_occupancy_grid_from_pgm(pgm_file_path)

    # Set resolution and origin
    resolution = 0.05
    origin_x, origin_y = 0.0, 0.0

    # Create the mock occupancy grid
    mock_grid = MockOccupancyGrid(width, height, resolution, origin_x, origin_y, data)

    # Initialize the pathfinder
    pathfinder = AStarPathfinder(mock_grid)

    # Extract coordinates from selected groceries
    coordinates = [(item['coordinate']['x'], item['coordinate']['y']) for item in selected_groceries]

    # Initialize the TSP solver and solve the problem
    tsp_solver = TSPSolver(pathfinder)
    sorted_coordinates = tsp_solver.solve(coordinates)

    # Plot the result
    plotter = Plotter(pathfinder)
    plotter.plot_solution(coordinates, sorted_coordinates)

def add_grocery_to_order(grocery_listbox, selected_listbox, groceries):
    selected_indices = grocery_listbox.curselection()
    for index in selected_indices:
        grocery = groceries[index]
        selected_listbox.insert(tk.END, grocery['label'])

def get_selected_groceries(selected_listbox, groceries):
    selected_grocery_labels = selected_listbox.get(0, tk.END)
    selected_groceries = [g for g in groceries if g['label'] in selected_grocery_labels]
    return selected_groceries

# Set up the GUI
root = tk.Tk()
root.title("Grocery Picker GUI")

# Load groceries from YAML file
yaml_file_path = 'C:/Users/ofekr/Desktop/autonomous car/code/tsp_solver/groceries.yaml'
pgm_file_path = 'C:/Users/ofekr/Desktop/autonomous car/code/tsp_solver/test_map.pgm'
groceries = load_groceries_from_yaml(yaml_file_path)

# Left Frame: Available Groceries
left_frame = tk.Frame(root)
left_frame.pack(side=tk.LEFT, padx=10, pady=10)

tk.Label(left_frame, text="Available Groceries").pack()
grocery_listbox = tk.Listbox(left_frame, selectmode=tk.MULTIPLE, width=50)
grocery_listbox.pack()

for grocery in groceries:
    grocery_listbox.insert(tk.END, grocery['label'])

# Right Frame: Selected Groceries
right_frame = tk.Frame(root)
right_frame.pack(side=tk.RIGHT, padx=10, pady=10)

tk.Label(right_frame, text="Selected Groceries").pack()
selected_listbox = tk.Listbox(right_frame, width=50)
selected_listbox.pack()

# Button to add groceries to the selected list
add_button = tk.Button(left_frame, text="Add to Order", command=lambda: add_grocery_to_order(grocery_listbox, selected_listbox, groceries))
add_button.pack(pady=5)

# Button to send the order and run the algorithm
send_order_button = tk.Button(right_frame, text="Send Order", command=lambda: run_algorithm(get_selected_groceries(selected_listbox, groceries), pgm_file_path))
send_order_button.pack(pady=5)

# Start the GUI loop
root.mainloop()
