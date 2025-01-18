import os
import glob
import json
import numpy as np
import open3d as o3d
import tkinter as tk
from tkinter import scrolledtext
from tkinter import filedialog
import threading

class PointCloudVisualizer:
    def __init__(self, root, directory_path):
        self.root = root
        self.root.title("Point Cloud Visualizer")
        # 设置文件夹路径
        self.directory_path = directory_path
        # 创建左侧设置框架
        self.settings_frame = tk.Frame(self.root)
        self.settings_frame.pack(side=tk.LEFT, padx=10, pady=10)

        # 创建文件路径输入框
        self.file_path_entry = tk.Entry(self.settings_frame, width=50)
        self.file_path_entry.pack(pady=5)
        self.file_path_entry.insert(0, directory_path)  # 插入默认路径

        # 创建浏览按钮
        self.browse_button = tk.Button(self.settings_frame, text="Browse", command=self.browse_files)
        self.browse_button.pack(pady=5)

        # 创建加载文件按钮
        self.load_button = tk.Button(self.settings_frame, text="Load Point Cloud", command=self.load_point_cloud)
        self.load_button.pack(pady=5)

        self.visualize_button = tk.Button(self.settings_frame, text="Start Visualizer", command=self.start_visualizer_thread)
        self.visualize_button.pack(pady=5)

        # 创建清除选择按钮
        self.clear_button = tk.Button(self.settings_frame, text="Clear Selected Points", command=self.clear_picked_points)
        self.clear_button.pack(pady=5)

        # 创建文本框用于显示输出
        self.output_text = scrolledtext.ScrolledText(self.root, width=100, height=20)
        self.output_text.pack(side=tk.RIGHT, padx=10, pady=10)

        self.pcd = []
        self.point_cloud = None
        self.vis = None

    def browse_files(self):
        """打开文件对话框以选择路径"""
        directory_path = filedialog.askdirectory()
        if directory_path:
            self.file_path_entry.delete(0, tk.END)  # 清空当前路径
            self.file_path_entry.insert(0, directory_path)  # 插入新路径

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
                    self.output_text.insert(tk.END, f"Error decoding JSON: {json_data}\n")
                    self.output_text.see(tk.END)  # 滚动到最新输出
        return np.array(points)

    def start_visualizer_thread(self):
        self.visualizer_thread = threading.Thread(target=self.run_visualizer, daemon=True)
        self.visualizer_thread.start()

    def run_visualizer(self):
        """运行可视化器"""
        self.vis = o3d.visualization.VisualizerWithVertexSelection()
        self.vis.create_window("Open3D - PointCloudVisualizer", width=800, height=600)

        # 在 Tkinter 中显示点云
        allcloud = np.array(self.pcd)
        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud.points = o3d.utility.Vector3dVector(allcloud)
        self.vis.add_geometry(self.point_cloud)

        # 注册选择变化的回调函数
        self.vis.register_selection_changed_callback(self.selection_changed_callback)
        self.vis.register_selection_moved_callback(self.selection_moved_callback)

        while True:
            try:
                self.vis.update_geometry(self.point_cloud)
                self.vis.poll_events()
                self.vis.update_renderer()
            except Exception as e:
                self.output_text.insert(tk.END, f"Visualizer error: {e}\n")
                break  # 退出循环以防止崩溃

    def clear_picked_points(self):
        if self.vis:
            try:
                self.vis.clear_picked_points()
                self.output_text.insert(tk.END, "Cleared selected points.\n")
                self.output_text.see(tk.END)  # 滚动到最新输出
            except Exception as e:
                self.output_text.insert(tk.END, f"Error clearing selected points: {e}\n")
                self.output_text.see(tk.END)  # 滚动到最新输出

    def selection_moved_callback(self):
        self.vis.clear_picked_points()

    def selection_changed_callback(self):
        # 获取当前选择的点
        picked_points = self.vis.get_picked_points()
        if picked_points:
            point_index = picked_points[-1].index
            point_coordinates = np.asarray(self.point_cloud.points)[point_index]
            self.output_text.insert(tk.END, f"Selected point index: {point_index}, coordinates[ x:{point_coordinates[0]}, y:{point_coordinates[1]}, z:{point_coordinates[2]}]\n")
            self.output_text.see(tk.END)  # 滚动到最新输出

    def load_point_cloud(self):
        """加载点云数据并启动可视化线程"""
        directory_path = self.file_path_entry.get()
        file_pattern = os.path.join(directory_path, "TestClouds*")
        files = glob.glob(file_pattern)
        files.sort()

        for file in files:
            self.output_text.insert(tk.END, f"Processing file: {file}\n")
            points = self.parse_file(file)
            self.output_text.insert(tk.END, f"Number of points in file {file}: {len(points)}\n")
            if len(points) <= 0:
                continue
        self.output_text.see(tk.END)  # 滚动到最新输出

if __name__ == "__main__":
    # 配置文件路径
    directory_path = "C:\\Users\\xmcchv\\Desktop\\宁东\\ndpython\\CloudsTest--------0118-----2"
    # 初始化可视化界面
    root = tk.Tk()
    viewer = PointCloudVisualizer(root, directory_path)
    root.mainloop()
