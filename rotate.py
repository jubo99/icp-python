import open3d as o3d
import numpy as np

import open3d as o3d
import numpy as np


def rotate_point_cloud(point_cloud, rotation_matrix):
    """
    Rotate a point cloud using a given rotation matrix.

    Parameters:
    - point_cloud: Open3D point cloud object
    - rotation_matrix: 3x3 rotation matrix

    Returns:
    - rotated_point_cloud: Rotated Open3D point cloud object
    """
    rotated_points = np.dot(point_cloud.points, rotation_matrix.T)
    rotated_point_cloud = o3d.geometry.PointCloud()
    rotated_point_cloud.points = o3d.utility.Vector3dVector(rotated_points)
    return rotated_point_cloud


def main():
    # Load point cloud from file
    input_file_path = "bmw_manifold_ply_1/BMW Manifold.ply"
    point_cloud = o3d.io.read_point_cloud(input_file_path)

    # Define rotation matrix (example: 45 degrees around the Z-axis)
    angle_degrees = 10.0
    angle_radians = np.radians(angle_degrees)
    rotation_matrix = np.array([[np.cos(angle_radians), -np.sin(angle_radians), 0],
                                [np.sin(angle_radians), np.cos(angle_radians), 0],
                                [0, 0, 1]])

    # Rotate point cloud
    rotated_point_cloud = rotate_point_cloud(point_cloud, rotation_matrix)

    # Save rotated point cloud to a new file
    output_file_path = "out.ply"
    o3d.io.write_point_cloud(output_file_path, rotated_point_cloud)


if __name__ == "__main__":
    main()
