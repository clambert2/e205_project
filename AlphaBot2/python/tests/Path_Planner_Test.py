import heapq
import csv
import numpy as np
import matplotlib.pyplot as plt

def heuristic(point, goal):
    # Manhattan distance heuristic
    return abs(point[0] - goal[0]) + abs(point[1] - goal[1])

def astar_with_turn_penalty(grid, start, goal, turn_penalty=5):
    open_set = []
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    heapq.heappush(open_set, (0, start, None))  # (f, current_pos, previous_direction)

    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current, prev_dir = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for dx, dy in directions:
            x, y = current[0] + dx, current[1] + dy
            neighbor = (x, y)
            direction = (dx, dy)

            if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 1:
                turn_cost = turn_penalty if prev_dir and direction != prev_dir else 0
                tentative_g = g_score[current] + 1 + turn_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor, direction))
                    came_from[neighbor] = current

    return None


# Example usage:
grid = np.loadtxt('report_frontier_0.csv', delimiter=',')

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

path = astar_with_turn_penalty(grid, start, goal)
print("Shortest path:", path)

path_x = []
path_y = []
for point in path:
    path_x.append(point[0])
    path_y.append(point[1])


# Plot results
plt.figure(figsize=(8, 8))
plt.imshow(grid, cmap='gray', origin='lower')
plt.plot(path_y, path_x, c='red', label='Path')
#plt.scatter(frontier_points[:, 1], frontier_points[:, 0], c='red', s=5, label='Frontier')
#plt.title('Frontier Detection from CSV')
plt.title('A* Path Planning')
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.xlim(35, 65)
plt.ylim(40, 70)
plt.grid(False)
plt.show()
