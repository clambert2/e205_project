import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

# Create square shape
def create_square(center=(0, 0), size=1.0, angle=0):
    half = size / 2
    square = np.array([
        [-half, -half],
        [ half, -half],
        [ half,  half],
        [-half,  half]
    ])
    # Rotation
    R = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]
    ])
    rotated = square @ R.T + np.array(center)
    return rotated

# Generate source and target
target = create_square(center=(0, 0), size=1.0, angle=0)
source = create_square(center=(0.1, 0.1), size=1.0, angle=np.deg2rad(15))

# Duplicate points to simulate a denser scan (optional)
target = np.repeat(target, 20, axis=0) + np.random.normal(0, 0.005, (80, 2))
source = np.repeat(source, 20, axis=0) + np.random.normal(0, 0.005, (80, 2))

# Plot before  
plt.scatter(target[:, 0], target[:, 1], label='Target', alpha=0.6)
plt.scatter(source[:, 0], source[:, 1], label='Source', alpha=0.6)
plt.legend()
plt.axis('equal')
plt.title("Before ICP")
plt.show()

# Convert to Open3D point clouds (add Z=0 for 2D)
def to_pcd(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.c_[points, np.zeros(len(points))])
    return pcd

pcd_target = to_pcd(target)
pcd_source = to_pcd(source)

# Run ICP
threshold = 0.2
trans_init = np.eye(4)
reg = o3d.pipelines.registration.registration_icp(
    pcd_source, pcd_target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

# Apply transformation
source_transformed = np.asarray(pcd_source.points)
T = reg.transformation
source_transformed = (T[:3, :3] @ source_transformed.T + T[:3, [3]]).T

# Plot after ICP
plt.scatter(target[:, 0], target[:, 1], label='Target', alpha=0.6)
plt.scatter(source_transformed[:, 0], source_transformed[:, 1], label='Aligned Source', alpha=0.6)
plt.legend()
plt.axis('equal')
plt.title("After ICP")
plt.show()

print("Transformation Matrix:")
print(reg.transformation)
