import heapq
import csv
import numpy as np
import matplotlib.pyplot as plt

def heuristic(point, goal):
    # Manhattan distance heuristic
    return abs(point[0] - goal[0]) + abs(point[1] - goal[1])

def astar(grid, start, goal):
    open_set = []

    # Priority queue with (F-score, node)
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            # Reconstruct the path and return
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            x, y = current[0] + dx, current[1] + dy
            neighbor = (x, y)

            # Assuming uniform cost for each step
            tentative_g = g_score[current] + 1
            
            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 1:
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
                    came_from[neighbor] = current
    
    return None  # No path found

# Example usage:
grid = np.loadtxt('occupancy_grid_with_end_3.csv', delimiter=',')
'''datafile = open('/Users/jaspercox/Downloads/occupancy_grid.csv', 'r')
datareader = csv.reader(datafile, delimiter=',')
grid = []
for row in datareader:
    grid.append(row) '''
##grid = [
#    [0, 0, 0, 0],
#    [0, 1, 1, 0],
#    [0, 1, 0, 0],
#    [0, 0, 0, 0]
#]

for i in range(0, len(grid)):
    for j in range(0, len(grid[0])):
        if grid[i][j] == 2:
            start = (i, j)
            grid[i][j] = 1

for i in range(0, len(grid)):
    for j in range(0, len(grid[0])):
        if grid[i][j] == 3:
            goal = (i, j)
            grid[i][j] = 1



'''start = (18,26)
goal = (28,32)'''

path = astar(grid, start, goal)
print("Shortest path:", path)

path_x = []
path_y = []
for point in path:
    path_x.append(point[0])
    path_y.append(point[1])


# Plot results
plt.figure(figsize=(8, 8))
plt.imshow(grid, cmap='gray', origin='lower')
plt.scatter(path_y, path_x, c='red', s=5, label='Path')
#plt.scatter(frontier_points[:, 1], frontier_points[:, 0], c='red', s=5, label='Frontier')
#plt.title('Frontier Detection from CSV')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(False)
plt.show()