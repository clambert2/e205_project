import math
import numpy as np

def rotation_matrix_from_vectors(v1, v2):
    """
    Returns the rotation matrix that aligns v1 to v2
    using Rodrigues' rotation formula.
    """
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    cross = np.cross(v1, v2)
    dot = np.dot(v1, v2)
    norm_cross = np.linalg.norm(cross)

    if norm_cross == 0:
        return np.eye(3)  # Vectors are already aligned

    skew = np.array([
        [0, -cross[2], cross[1]],
        [cross[2], 0, -cross[0]],
        [-cross[1], cross[0], 0]
    ])

    R = np.eye(3) + skew + skew @ skew * ((1 - dot) / (norm_cross ** 2))
    return R

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
