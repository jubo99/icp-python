import time
import open3d as o3d
import numpy as np
import icp_git


input_file_path = "bmw_manifold_ply_1/BMW Manifold.ply"
pcd = o3d.io.read_point_cloud(input_file_path)
pcd_rot = o3d.io.read_point_cloud("out.ply")
# o3d.visualization.draw_geometries([pcd, pcd_rot])

# Convert Points to Numpy array
pcd_points = np.asanyarray(pcd.points)
pcd_rot_points = np.asanyarray(pcd_rot.points)
rot_length = pcd_rot_points.shape[0]
total_time = 0
# Run ICP
start = time.time()
T, distances, iterations = icp_git.icp(pcd_rot_points, pcd_points, tolerance=0.000001)
total_time += time.time() - start

# Make C a homogeneous representation of B
C = np.ones((rot_length, 4))
C[:, 0:3] = np.copy(pcd_rot_points)

# Transform C
C = np.dot(T, C.T).T

print('icp time: {:.3}'.format(total_time))

transformed_pcl = np.delete(C, 3, axis=1)

# Konvertierung zu einer Open3D-Punktwolke
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(transformed_pcl)

# Visualisierung der Punktwolke (optional)
o3d.visualization.draw_geometries([point_cloud, pcd])

print("Hi")


