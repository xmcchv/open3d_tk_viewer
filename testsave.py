import open3d as o3d
import numpy as np

# 创建一个简单的点云
points = np.random.rand(100, 3)  # 随机生成 100 个点
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)

# 设置要保存的中文文件名
file_name = "./pcd/中文文件名.ply"  # 中文文件名

# 保存点云
o3d.io.write_point_cloud(file_name, point_cloud)

print(f"点云数据已保存为 {file_name}")
