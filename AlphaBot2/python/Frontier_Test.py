import numpy as np
import matplotlib.pyplot as plt

'''
Frontier Search Guide

1. Find all white cells that are bordering the black cells.
    1.1. The white cells are 1 and the black cells are -1.


'''

# Load CSV file as occupancy grid
grid = np.loadtxt('occupancy_grid.csv', delimiter=',')

# Constants for cell types
UNKNOWN = 0
OCCUPIED = 127
FREE = 255

# Frontier mask: 1 for frontier, 0 otherwise
frontier_mask = np.zeros_like(grid, dtype=np.uint8)

# Get grid dimensions
rows, cols = grid.shape

# Check each cell (excluding the border to avoid index errors)
for y in range(1, rows - 1):
    for x in range(1, cols - 1):
        if grid[y, x] == FREE:
            neighborhood = grid[y-1:y+2, x-1:x+2].flatten()
            if UNKNOWN in neighborhood and OCCUPIED not in neighborhood:
                frontier_mask[y, x] = 1

# Get coordinates of frontier points
frontier_points = np.argwhere(frontier_mask == 1)

# Plot results
plt.figure(figsize=(8, 8))
plt.imshow(grid, cmap='gray', origin='lower')
plt.scatter(frontier_points[:, 1], frontier_points[:, 0], c='red', s=5, label='Frontier')
plt.title('Frontier Detection from CSV')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(False)
plt.show()
