import time
import open3d as o3d
import numpy as np
import icp_git
import json


# Read in config-file
with open('config.json', 'r') as file:
    config_data = json.load(file)

# Access Config Values
database_host = config_data['database']['host']
database_port = config_data['database']['port']
down_sampling_rate = config_data['down_sampling_rate']
noise_level = config_data['noise_level']
data_path_template = config_data['data_path_template']
data_path_recording = config_data['data_path_recording']
color = config_data['color']

#Read in PCLs
pcd = o3d.io.read_point_cloud(data_path_template)
pcd_rot = o3d.io.read_point_cloud(data_path_recording)


# color transformed pcl
pcd_rot.paint_uniform_color(color)
o3d.visualization.draw_geometries([pcd, pcd_rot])

# every x-th row will be taken for ICP
x = down_sampling_rate
# Convert Points to Numpy array
pcd_points = np.asanyarray(pcd.points)
pcd_points = pcd_points[::x, :]
pcd_rot_points = np.asanyarray(pcd_rot.points)
pcd_rot_points = pcd_rot_points[::x, :]
rot_length = pcd_rot_points.shape[0]

# Add random noise to all points
pcd_noise = pcd_rot_points + noise_level * np.random.randn(rot_length, 3)


total_time = 0
# Run ICP
start = time.time()
T, distances, iterations = icp_git.icp(pcd_noise, pcd_points, tolerance=0.001)
total_time += time.time() - start

# Make C a homogeneous representation of B
C = np.ones((rot_length, 4))
C[:, 0:3] = np.copy(pcd_noise)

# Transform C
C = np.dot(T, C.T).T

print('icp time: {:.3}'.format(total_time))

transformed_pcl = np.delete(C, 3, axis=1)

# Konvertierung zu einer Open3D-Punktwolke
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(transformed_pcl)


# color transformed pcl
point_cloud.paint_uniform_color(color)

# Visualisierung der Punktwolke (optional)
o3d.visualization.draw_geometries([point_cloud, pcd])

print("Job finished")


