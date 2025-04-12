import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the CSV data
df_0 = pd.read_csv('scan_corner_0.csv')
df_1 = pd.read_csv('scan_0.csv')
# df_2 = pd.read_csv('scan_corner_2.csv')
# df_3 = pd.read_csv('scan_corner_3.csv')

# Only take the first 500 rows
df_0 = df_0.head(500)
df_1 = df_1.head(500)
# df_2 = df_2.head(500)
# df_3 = df_3.head(500)

# Extract angles
angles_0 = df_0['Angle'].values
angles_1 = df_1['Angle'].values
# angles_2 = df_2['Angle'].values
# angles_3 = df_3['Angle'].values

# Extract distances
distances_0 = df_0['Distance'].values
distances_1 = df_1['Distance'].values
# distances_2 = df_2['Distance'].values
# distances_3 = df_3['Distance'].values

# Convert angles to radians
angles_rad_0 = np.deg2rad(angles_0)
angles_rad_1 = np.deg2rad(angles_1)
# angles_rad_2 = np.deg2rad(angles_2)
# angles_rad_3 = np.deg2rad(angles_3)

# Create a polar plot
plt.figure(figsize=(8, 8))
ax = plt.subplot(111, projection='polar')
ax.plot(angles_rad_0, distances_0, marker='o', linestyle='-', markersize=3, color='r')
ax.plot(angles_rad_1, distances_1, marker='o', linestyle='-', markersize=3, color='b')
# ax.plot(angles_rad_2, distances_2, marker='o', linestyle='-', markersize=3, color='g')
# ax.plot(angles_rad_3, distances_3, marker='o', linestyle='-', markersize=3, color='y')

# Label plot
ax.set_title("RPLidar Scan Data")
ax.set_xlabel("Angle (radians)")
ax.set_ylabel("Distance (meters)")
ax.set_ylim(0,  600)

# Show the plot
plt.show()
