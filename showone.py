# -*- coding: utf-8 -*-
import json
import numpy as np
import open3d as o3d

position_info = 0

def parse_file(file_path):
    global position_info
    points = []

    with open(file_path, 'r') as file:
        for line in file:
            # 分割大车位置和JSON部分
            parts = line.strip().split('---')
            if len(parts) < 2:
                continue
            
            # 解析大车位置
            position_info = float(parts[0].strip())
            print(f"行车位置: {position_info}")

            # 解析JSON部分
            json_data = parts[1].strip()
            try:
                json_objects = json.loads(json_data)
                for obj in json_objects:
                    #读取坐标和其他信息
                    x = float(obj['X'])
                    y = float(obj['Y'])
                    z = float(obj['Z'])

                    # alpha = float(obj['AngL'])/180*np.pi
                    # omega = float(obj['AngV'])/180*np.pi
                    # r = float(obj['V1']) / 1000
                    # # print(alpha,omega,r)
                    # # 计算点云坐标
                    # z = r * np.sin(omega) + position_info
                    # x = r * np.sin(alpha) * np.cos(omega) + 0.0635 * np.cos(alpha)
                    # y = r * np.cos(alpha) * np.cos(omega) + 0.0635 * np.sin(alpha)

                    # 计算点云坐标相对于大车位置
                    points.append([x, y, z])

            except json.JSONDecodeError:
                print("Error decoding JSON: {}".format(json_data))

    return np.array(points)

def visualize_point_cloud(points):
    # 创建Open3D点云对象
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # 可视化点云
    FOR1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, position_info])
    o3d.visualization.draw_geometries([point_cloud,FOR1])
    #保存点云为pcl文件
    o3d.io.write_point_cloud("./pcd/point_cloud.pcd", point_cloud)

if __name__ == "__main__":
    file_path = "C:\\Users\\xmcchv\\Desktop\\宁东\\ndpython\\CloudsTest-----0118--------11-1\\TestClouds2025-01-18-15-41-00"  # 替换为您的文件路径
    points = parse_file(file_path)
    
    if points.size > 0:
        visualize_point_cloud(points)
    else:
        print("No points to visualize.")
