import numpy as np
import open3d as o3d
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math

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

    print(f"Translation:")
    print(f"  X: {translation[0]:.2f}")
    print(f"  Y: {translation[1]:.2f}")
    print(f"  Z: {translation[2]:.2f}")
    print(f"Rotation around Z (yaw): {angle_deg:.2f} degrees")


# Load the CSV data
df_0 = pd.read_csv('lidar_move_and_scan_2.csv')
df_1 = pd.read_csv('lidar_move_and_scan_3.csv')

# Only take the first 500 rows
df_0 = df_0.head(500)
df_1 = df_1.head(500)

# Extract angles
angles_0 = df_0['Angle'].values
angles_1 = df_1['Angle'].values

# Extract distances
distances_0 = df_0['Distance'].values
distances_1 = df_1['Distance'].values

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

# Run ICP
THRESHOLD = 1000 # The maximum distance from point to point (set pretty high)
trans_init = np.array([[ 0.000000e+00, -1.000000e+00,  0.000000e+00,  0.000000e+00],
       [ 1.000000e+00,  6.123234e-17,  0.000000e+00,  0.000000e+00],
       [ 0.000000e+00,  0.000000e+00,  1.000000e+00,  0.000000e+00],
       [ 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00]])  # guess for transformation matrix
reg_p2p = o3d.pipelines.registration.registration_icp(
    source_pcd, target_pcd, THRESHOLD, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

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
