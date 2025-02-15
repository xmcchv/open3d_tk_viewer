import os
import glob
import json
import numpy as np
import open3d as o3d

print("Open3D version:", o3d.__version__)

class PointCloudVisualizer:
    def __init__(self):
        self.pcd = []
        self.point_cloud = None
        self.vis = None

    def parse_file(self, file_path):
        points = []
        
        with open(file_path, 'r') as file:
            for line in file: 
                parts = line.strip().split('---')  # 分割大车位置和JSON部分
                if len(parts) < 2:
                    continue
                
                json_data = parts[1].strip()
                try:
                    json_objects = json.loads(json_data)
                    for obj in json_objects:
                        x = obj['X']
                        y = obj['Y']
                        z = obj['Z']
                        points.append([x, y, z])
                        self.pcd.append([x, y, z])
                except json.JSONDecodeError:
                    print("Error decoding JSON: {}".format(json_data))
        return np.array(points)

    def visualize_point_cloud(self, points):
        if points.size == 0:
            print("No points to visualize.")
            return
        
        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud.points = o3d.utility.Vector3dVector(points)

        # 使用 VisualizerWithVertexSelection 进行可视化
        self.vis = o3d.visualization.VisualizerWithVertexSelection()
        self.vis.create_window("Open3D - PointCloudVisualizer", 1024, 768)
        self.vis.add_geometry(self.point_cloud)

        # 注册选择变化的回调函数
        self.vis.register_selection_changed_callback(self.selection_changed_callback)
        self.vis.register_selection_moved_callback(self.selection_moved_callback)
        # self.vis.register_selection_moving_callback(self.selection_moving_callback)
        # 注册按键事件的回调函数
        # self.vis.register_key_callback(ord('P'), self.clear_picked_point)
        self.vis.run()
        self.vis.destroy_window()
    
    def clear_picked_point(self):
        self.vis.clear_picked_points()

    # def selection_moving_callback(self):
    #     print("i'm moving")

    def selection_moved_callback(self):
        self.vis.clear_picked_points()

    def selection_changed_callback(self):
        # 获取当前选择的点
        picked_points = self.vis.get_picked_points()
        if picked_points:
            point_index = picked_points[-1].index
            point_coordinates = np.asarray(self.point_cloud.points)[point_index]
            print(f"Selected point index: {point_index}, coordinates[ x:{point_coordinates[0]}, y:{point_coordinates[1]}, z:{point_coordinates[2]}]")

    def run(self):
        # 配置文件路径
        directory_path = "C:\\Users\\xmcchv\\Desktop\\宁东\\ndpython\\CloudsTest------0117-----7"
        file_pattern = os.path.join(directory_path, "TestClouds*")
        files = glob.glob(file_pattern)
        files.sort()

        # 循环解析文件
        for file in files:
            print(f"Processing file: {file}")
            points = self.parse_file(file)
            print(f"Number of points in file {file}: {len(points)}")
            if len(points) <= 0:
                continue

        # 显示整体点云
        allcloud = np.array(self.pcd)
        self.visualize_point_cloud(allcloud)

if __name__ == "__main__":
    # 初始化可视化界面
    viewer = PointCloudVisualizer()
    viewer.run()
