import numpy as np
import matplotlib.pyplot as plt
from skimage.draw import line
import pandas as pd

# === Parameters ===
resolution = 0.05         # meters per cell
grid_size = 50           # 100x100 cells (small for testing)
origin = (grid_size // 2, grid_size // 2)  # LIDAR at center

# === Load CSV ===
# CSV format: confidence, angle (degrees), distance (meters)
df = pd.read_csv("lidar_move_and_scan_1.csv", header=None, names=["Confidence", "Angle", "Dist"])
# Convert mm to meters (if needed)

# Convert strings to numbers and drop bad rows
df = df.apply(pd.to_numeric, errors='coerce').dropna()
df["Dist"] = df["Dist"] / 1000.0  # Convert distance to meters

# === Convert polar to Cartesian ===
angles_rad = np.deg2rad(df["Angle"].values)
xs = df["Dist"].values * np.cos(angles_rad)
ys = df["Dist"].values * np.sin(angles_rad)
points = np.vstack((xs, ys)).T

# === Convert to grid coordinates ===
grid_points = np.round(points / resolution).astype(int)
grid_points += origin  # shift so origin is center


# === Initialize grid ===
grid = -np.ones((grid_size, grid_size), dtype=np.int8)  # -1 = unknown

# === Ray trace for free space ===
for gx, gy in grid_points:
    if 0 <= gx < grid_size and 0 <= gy < grid_size:
        rr, cc = line(origin[1], origin[0], gy, gx)  # Swap x and y here to fix rotation
        rr = np.clip(rr, 0, grid_size - 1)  # Ensure within bounds
        cc = np.clip(cc, 0, grid_size - 1)  # Ensure within bounds
        
        grid[rr, cc] = 1  # Mark free space

# === Mark occupied cells ===
for gx, gy in grid_points:
    if 0 <= gx < grid_size and 0 <= gy < grid_size:
        grid[gy, gx] = 0  # occupied

# === Print Information ===
print("Max coords:", grid_points.max(axis=0))
print("Min coords:", grid_points.min(axis=0))
print("Any occupied cells?", np.any(grid == 1))
print("Any free cells?", np.any(grid == 0))

print("Min angle:", df["Angle"].min())
print("Max angle:", df["Angle"].max())

# === Visualize Grid with Rays and Points ===
plt.figure(figsize=(8, 8))
plt.imshow(grid.T, cmap="grey", origin="lower")
plt.plot(origin[0], origin[1], "ro", label="Robot Position")
plt.title("Occupancy Grid")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(False)
plt.legend()
plt.show()

# === Load second CSV ===
df2 = pd.read_csv("combined_points.csv", header=None, names=["X", "Y"])
df2 = df2.apply(pd.to_numeric, errors='coerce').dropna()
points2 = df2[["X", "Y"]].values
points2 = points2 / 1000.0  # Convert to meters

# === Define the robot's new position for the second scan (dx, dy in meters) ===
dx, dy = 0 * resolution, 7 * resolution  # e.g., robot moved 5 cells right and 3 up

# Define the second robot's new position in world coordinates
origin2_world = np.array([dx, dy])  # This is the new robot position in meters

# Convert the new origin2 position to grid coordinates
origin2_grid = np.round(origin2_world / resolution).astype(int) + origin  # Adjust to grid coordinates

print("New origin2 grid coordinates:", origin2_grid)

# === The points from scan 2 remain the same, no translation of the scan points themselves ===
# Convert scan 2 points to grid coordinates relative to the first origin
grid_points2 = np.round(points2 / resolution).astype(int)
grid_points2 += origin  # shift so origin is center

# === Ray trace for free space (from the second origin in the new grid) ===
for gx, gy in grid_points2:
    if 0 <= gx < grid_size and 0 <= gy < grid_size:
        rr, cc = line(origin2_grid[0], origin2_grid[1], gy, gx)  # Use new origin2_grid
        print("gy, gx:", gy, gx)
        print("origin2_grid:", origin2_grid[1], " ",origin2_grid[0])
        rr = np.clip(rr, 0, grid_size - 1)
        cc = np.clip(cc, 0, grid_size - 1)
        grid[rr, cc] = 1  # Mark free space

# === Mark occupied cells ===
for gx, gy in grid_points2:
    if 0 <= gx < grid_size and 0 <= gy < grid_size:
        grid[gy, gx] = 0  # Mark occupied

# === Visualize updated grid ===
plt.figure(figsize=(8, 8))
plt.imshow(grid.T, cmap="grey", origin="lower")
plt.plot(origin[0], origin[1], "ro", label="Robot Position 1")
plt.plot(origin2_grid[0], origin2_grid[1], "bo", label="Robot Position 2")
# plt.scatter(grid_points2[:, 1], grid_points2[:, 0], s=10, c="green", label="Scan 2 Points")
# plt.scatter(grid_points[:, 1], grid_points[:, 0], s=10, c="red", label="Scan 1 Points")
plt.title("Occupancy Grid with Two Scans")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(False)
plt.legend()
plt.show()
