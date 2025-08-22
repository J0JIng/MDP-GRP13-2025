from algorithm.algo import find_path

# Define a test map
start = (0, 0)
goal = (4, 4)
obstacles = [(2, 2), (3, 2)]
grid_size = (5, 5)

path = find_path(start, goal, obstacles, grid_size)
print("Planned path:", path)
