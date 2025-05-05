import numpy as np
import matplotlib.pyplot as plt

'''
Frontier Search Guide

1. Find all white cells that are bordering the black cells.
    1.1. The white cells are 1 and the black cells are -1.


'''

# Load CSV file as occupancy grid
grid = np.loadtxt('report_grid_0.csv', delimiter=',')

print(grid)

for i in range(len(grid)):
    for j in range(len(grid[0])):
        if grid[i][j] == 1:
            # Check if any of the 4 sides are touching a black cell (-1)
            if (i > 0 and grid[i-1][j] == -1) or \
               (i < len(grid)-1 and grid[i+1][j] == -1) or \
               (j > 0 and grid[i][j-1] == -1) or \
               (j < len(grid[0])-1 and grid[i][j+1] == -1):
                # Mark the cell as a frontier point (2)
                grid[i][j] = 4


# Get coordinates of frontier points
frontier_points = np.argwhere(grid == 4)

frontier_set = set(map(tuple, frontier_points))  # For O(1) lookups

visited = set()
longest_group = []

# 8-connected directions (or use 8 for diagonals too)
directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (1,1), (-1, -1), (1, -1), (-1, 1)]

def dfs(start):
    stack = [start]
    group = []
    while stack:
        current = stack.pop()
        if current in visited:
            continue
        visited.add(current)
        group.append(current)
        x, y = current
        for dy, dx in directions:
            neighbor = (x + dx, y + dy)
            if neighbor in frontier_set and neighbor not in visited:
                stack.append(neighbor)
    return group

# Search all points for the longest connected group
for point in frontier_set:
    if point not in visited:
        group = dfs(point)
        if len(group) > len(longest_group):
            longest_group = group

print(f"Longest frontier length: {len(longest_group)}")
print("Coordinates:")
print(longest_group)

# Sort the group to ensure consistent order (optional but helps for visual logic)
longest_group_sorted = sorted(longest_group)

# Get the middle point by index
mid_index = len(longest_group_sorted) // 2
middle_point = longest_group_sorted[mid_index]

print(f"Middle point of longest frontier (by index): {middle_point}")



#set the frotier cells back to 1
for point in frontier_points:
    grid[point[0]][point[1]] = 1

    # set the middle point to 3 in the grid
grid[middle_point[0]][middle_point[1]] = 3

# make all numbers integers before saving
grid = grid.astype(int)

# save the modified grid to a new CSV file
np.savetxt('report_frontier_0.csv', grid, delimiter=',')

# Plot results
plt.figure(figsize=(8, 8))
plt.imshow(grid, cmap='gray', origin='lower')
plt.scatter(frontier_points[:, 1], frontier_points[:, 0], c='red', s=5, label='Frontier')
plt.scatter(middle_point[1], middle_point[0], c='blue', s=5, label='Middle Point', edgecolor='black')
plt.title('Frontier Detection from CSV')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(False)
plt.show()
