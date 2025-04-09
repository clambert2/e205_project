import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the CSV data
df = pd.read_csv('scan_1.csv')  # replace with your CSV file path

df = df.head(500)  # Limit to the first 1000 rows for plotting

# Extract the angle and distance (assuming the columns are named 'angle' and 'distance')
angles = df['Angle'].values  # in degrees
distances = df['Distance'].values  # in meters

# print(angles)
# print(distances)

# Convert angles to radians
angles_rad = np.deg2rad(angles)

# Create a polar plot
plt.figure(figsize=(8, 8))
ax = plt.subplot(111, projection='polar')
ax.plot(angles_rad, distances, marker='o', linestyle='-', markersize=3)

# Optional: Customize the plot
ax.set_title("RPLidar Scan Data")
ax.set_xlabel("Angle (radians)")
ax.set_ylabel("Distance (meters)")
ax.set_ylim(0,  600)  # Adjust the radius for better visibility

# Show the plot
plt.show()
