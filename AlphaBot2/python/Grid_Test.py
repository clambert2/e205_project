import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from skimage.draw import line

# === Parameters ===
resolution = 0.05        # meters per grid cell
grid_size = 100          # 100x100 grid
origin = (grid_size // 2, grid_size // 2)  # center of the grid

# === Load point cloud CSV ===
# Format: confidence, angle (degrees), distance (mm)
df = pd.read_csv("report_scan_1.csv", header=None, names=["Confidence", "Angle", "Dist"])
df = df.apply(pd.to_numeric, errors='coerce').dropna()
df["Dist"] = df["Dist"] / 1000.0  # Convert mm to meters

# === Convert polar to Cartesian ===
angles_rad = np.deg2rad(df["Angle"].values)
xs = df["Dist"].values * np.cos(angles_rad)
ys = df["Dist"].values * np.sin(angles_rad)
points = np.vstack((xs, ys)).T

# === Convert to grid coordinates ===
grid_points = np.round(points / resolution).astype(int)
grid_points += origin  # shift origin to center

# === Initialize grid ===
grid = -np.ones((grid_size, grid_size), dtype=np.int8)  # -1 = unknown

# === Ray trace for free space ===
for gx, gy in grid_points:
    if 0 <= gx < grid_size and 0 <= gy < grid_size:
        rr, cc = line(origin[1], origin[0], gx, gy)  # note: y, x
        rr = np.clip(rr, 0, grid_size - 1)
        cc = np.clip(cc, 0, grid_size - 1)
        grid[rr, cc] = 1  # mark free space

# === Mark occupied points ===
for gx, gy in grid_points:
    if 0 <= gx < grid_size and 0 <= gy < grid_size:
        grid[gx, gy] = 0  # mark occupied

# === Set robot origin ===
grid[origin[1], origin[0]] = 2  # mark origin with a 2

# === Save to CSV ===
np.savetxt("report_grid_1.csv", grid, fmt="%d", delimiter=",")

# === Plot ===
plt.figure(figsize=(8, 8))
plt.imshow(grid, cmap="gray", origin="lower")
# plt.plot(origin[0], origin[1], "ro", label="Robot Origin")
plt.title("Occupancy Grid")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.grid(False)
plt.show()
