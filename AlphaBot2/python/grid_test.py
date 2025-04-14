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
df = pd.read_csv("lidar_move_and_scan_0.csv", header=None, names=["Confidence", "Angle", "Dist"])
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
        
        grid[rr, cc] = 0  # Mark free space

# === Mark occupied cells ===
for gx, gy in grid_points:
    if 0 <= gx < grid_size and 0 <= gy < grid_size:
        grid[gy, gx] = 1  # occupied

# === Print Information ===
print("Max coords:", grid_points.max(axis=0))
print("Min coords:", grid_points.min(axis=0))
print("Any occupied cells?", np.any(grid == 1))
print("Any free cells?", np.any(grid == 0))

print("Min angle:", df["Angle"].min())
print("Max angle:", df["Angle"].max())

# === Visualize Grid with Rays and Points ===
plt.figure(figsize=(8, 8))
plt.imshow(grid.T, cmap="gray", origin="lower")
plt.scatter(grid_points[:, 1], grid_points[:, 0], color="red", label="Grid Points", alpha=0.6)  # Swap x and y for the scatter plot
plt.title("Occupancy Grid with Ray Tracing and LIDAR Points")
plt.xlabel("X (grid)")
plt.ylabel("Y (grid)")
plt.grid(False)
plt.legend()
plt.show()
