import numpy as np
import open3d as o3d
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
from scipy.signal import butter, filtfilt


# Load the CSV data
df_lidar_0 = pd.read_csv('lidar_move_and_scan_2.csv')
df_lidar_1 = pd.read_csv('lidar_move_and_scan_1.csv')
df_accel = pd.read_csv('accel_move_and_scan_1.csv')

# Extract angles
angles_0 = df_lidar_0['Angle'].values
angles_1 = df_lidar_1['Angle'].values

# Extract distances
distances_0 = df_lidar_0['Distance'].values
distances_1 = df_lidar_1['Distance'].values

# Convert polar (r, theta) to Cartesian
def polar_to_cartesian(r, theta):
    '''Convert polar coordinates to Cartesian coordinates.'''
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return np.vstack((x, y)).T

# Convert to Cartesian coordinates
source_points = polar_to_cartesian(distances_0, np.deg2rad(angles_0))
target_points = polar_to_cartesian(distances_1, np.deg2rad(angles_1))

# Convert to Open3D format Need to add a z coordinate dimension for it to work
source_pcd = o3d.geometry.PointCloud()
target_pcd = o3d.geometry.PointCloud()
source_pcd.points = o3d.utility.Vector3dVector(
    np.c_[source_points, np.zeros(len(source_points))]
)
target_pcd.points = o3d.utility.Vector3dVector(
    np.c_[target_points, np.zeros(len(target_points))]
)

# Apply a butterworth filter to the accelerometer data
def butter_lowpass_filter(data, cutoff, order=5):
    fs = 1 / (data['Time'][1] - data['Time'][0])  # Sampling frequency
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)

    filtered_data =  {
        'Accel_X': filtfilt(b, a, data['Accel_X']),
        'Accel_Y': filtfilt(b, a, data['Accel_Y']),
        'Accel_Z': filtfilt(b, a, data['Accel_Z']),
    }

    return pd.DataFrame(filtered_data)

butter_filtered_df = butter_lowpass_filter(df_accel, cutoff=0.99, order=2)

# Integrate the accelerometer data
velocity = [np.array([0.0, 0.0, 0.0])]
position = [np.array([0.0, 0.0, 0.0])]
accel_data = butter_filtered_df[['Accel_X', 'Accel_Y', 'Accel_Z']].values
time_data = df_accel['Time'].values
for i in range(1, len(accel_data)):
    dt = time_data[i] - time_data[i - 1]
    accel = accel_data[i] - np.array([0, 0, 9.81])  # Remove gravity
    velocity.append(velocity[-1] + accel * dt)
    position.append(position[-1] + velocity[-1] * dt)

print(f"Final velocity: {velocity[-1]}")
print(f"Final position: {position[-1]*1000}")

# plot each column of the accelerometer data vs time (raw from csv)
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(time_data, accel_data[:, 0], label='Accel_X')
plt.title('Accelerometer Data')
plt.ylabel('Accel_X')
plt.legend()
plt.subplot(3, 1, 2)
plt.plot(time_data, accel_data[:, 1], label='Accel_Y')
plt.ylabel('Accel_Y')
plt.legend()
plt.subplot(3, 1, 3)
plt.plot(time_data, accel_data[:, 2], label='Accel_Z')
plt.ylabel('Accel_Z')
plt.xlabel('Time (s)')
plt.legend()
plt.tight_layout()
plt.show()

# Run ICP
THRESHOLD = 1000 # The maximum distance from point to point (set pretty high)
# trans_init = np.array([[ 0.000000e+00, 1.000000e+00,  0.000000e+00,  0.000000e+00],
#                        [ -1.000000e+00,  0.000000e+00,  0.000000e+00,  0.000000e+00],
#                        [ 0.000000e+00,  0.000000e+00,  1.000000e+00,  0.000000e+00],
#                        [ 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00]])
def z_rotation_matrix(degrees):
    theta = np.radians(degrees)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    
    return np.array([
        [ cos_theta, sin_theta, 0.0, 0.0],
        [-sin_theta, cos_theta, 0.0, 0.0],
        [ 0.0      , 0.0      , 1.0, 0.0],
        [ 0.0      , 0.0      , 0.0, 1.0]
    ])

trans_init = z_rotation_matrix(110)  # Rotate -90 degrees around Z axis
# trans_init = np.eye(4)
# trans_init[0:3, 3] = position[-1]*1000  # Set translation to final position
reg_p2p = o3d.pipelines.registration.registration_icp(
    source_pcd, target_pcd, THRESHOLD, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)


def interpret_and_print_transform(matrix):
    """
    Interprets a 4x4 homogeneous transformation matrix and prints:
    - Translation in X, Y, Z
    - Rotation around Z axis (in degrees)
    
    Args:
        matrix (np.ndarray): A 4x4 numpy array.
    """
    if matrix.shape != (4, 4):
        raise ValueError("Expected a 4x4 matrix.")

    translation = matrix[:3, 3]
    rotation_matrix = matrix[:3, :3]

    # Since this is 2D motion, we assume rotation about Z
    angle_rad = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    angle_deg = math.degrees(angle_rad)

    print("Translation:")
    print(f"  X: {translation[0]:.2f}")
    print(f"  Y: {translation[1]:.2f}")
    print(f"  Z: {translation[2]:.2f}")
    print(f"Rotation around Z (yaw): {angle_deg:.2f} degrees")


# Print the transformation matrix
interpret_and_print_transform(reg_p2p.transformation)

# Converts source into homogeneous coordinates for transformation
# Format: (x, y, z, w) [where w=1] as a scaling factor
source_points_3d = np.asarray(source_pcd.points)
source_homogeneous = np.hstack((source_points_3d,
                                np.ones((source_points_3d.shape[0],
                                1
))))

# Apply transformation matrix and remove w coordinate
T = reg_p2p.transformation
source_transformed = (T @ source_homogeneous.T).T[:, :3]  # keep only x, y, z

plt.scatter(*source_points.T, label='Original Source',
            alpha=0.5
)
plt.scatter(*target_points.T, label='Target',
            alpha=0.5
)
plt.scatter(source_transformed[:, 0],
            source_transformed[:, 1],
            label='Transformed Source',
            alpha=0.5
)
plt.axis('equal')
plt.legend()
plt.show()
# Save the combined point cloud to a csv file

combined_array = np.vstack((source_transformed[:, :2]))
np.savetxt("combined_points.csv", combined_array, delimiter=",", fmt="%.6f")
