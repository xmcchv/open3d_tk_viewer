import open3d as o3d
import numpy as np

# # 加载点云
# point_cloud = o3d.io.read_point_cloud("pcd\\CloudsTest1.segmented.ply")  # 替换为你的点云文件路径

# # 将点云转换为 numpy 数组
# points = np.asarray(point_cloud.points)

# # 计算 Z 方向的平均值
# z_mean = np.mean(points[:, 2])

# print(f"Z方向的平均值: {z_mean}")

print(np.tan(np.deg2rad(8.99)))
print(10.51*np.tan(np.deg2rad(8.99)))