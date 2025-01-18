import os
import glob
import json
import numpy as np
import open3d as o3d
import time

position_info = 0
pcd = []
def parse_file(file_path):
    global position_info
    points = []
    
    with open(file_path, 'r') as file:
        for line in file: 
            parts = line.strip().split('---')  # 分割大车位置和JSON部分
            if len(parts) < 2:
                continue
            
            position_info = float(parts[0].strip())
            print(f"行车位置: {position_info}")
            
            json_data = parts[1].strip()
            try:
                json_objects = json.loads(json_data)
                for obj in json_objects:
                    x = obj['X']
                    y = obj['Y']
                    z = obj['Z']
                    points.append([x, y, z])
                    pcd.append([x,y,z])
                    
                    # angv = float(obj['AngV'])
                    # angl = float(obj['AngL'])
                    # # if angl < 0:
                    # #     angl = angl + 360
                    # alpha = angl/180*np.pi
                    # omega = angv/180*np.pi
                    # r = float(obj['V1']) / 1000
                    # # print(alpha,omega,r)
                    # # 计算点云坐标
                    # z = r * np.sin(omega) + position_info
                    # x = r * np.sin(alpha) * np.cos(omega) + 0.0635 * np.cos(alpha)
                    # y = r * np.cos(alpha) * np.cos(omega) + 0.0635 * np.sin(alpha)

                    # points.append([x, y, z])
                    # pcd.append([x,y,z])
            except json.JSONDecodeError:
                print("Error decoding JSON: {}".format(json_data))

    return np.array(points)

def visualize_point_cloud(points):
    if points.size == 0:
        print("No points to visualize.")
        return
    
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # 使用Open3D的可视化窗口
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(point_cloud)

    FOR1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, position_info])
    vis.add_geometry(FOR1)
    
    # 显示可视化窗口
    vis.run()
    

if __name__ == "__main__":
    directory_path = "C:\\Users\\13576\\Desktop\\ndpython\\CloudsTest------0115-----05"
    file_pattern = os.path.join(directory_path, "TestClouds*")
    files = glob.glob(file_pattern)
    files.sort()
    for file in files:
        print(f"Processing file: {file}")
        points = parse_file(file)
        print(f"Updating point cloud with {len(points)} points.") 
        visualize_point_cloud(points)
