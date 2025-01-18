import os
import glob
import json
import numpy as np
import open3d as o3d
import tkinter as tk
from tkinter import scrolledtext
from tkinter import filedialog
import threading


"""
    @param directory_path 配置文件路径
    SHIFT+左键选点 右键清除选点
"""
directory_path = "C:\\Users\\xmcchv\\Desktop\\宁东\\ndpython\\CloudsTest---------0118----8"

class PointCloudVisualizer:
    def __init__(self, root, directory_path):
        self.root = root
        self.root.geometry("960x480")
        self.root.title("Point Cloud Visualizer")
        # 设置文件夹路径
        self.directory_path = directory_path

        # 创建文本框用于显示输出
        self.output_text = scrolledtext.ScrolledText(self.root, width=100, height=20, 
                                                      font=("Courier New", 20),  # 设置字体
                                                      bg="white",  # 设置背景色
                                                      fg="black",  # 设置前景色
                                                      insertbackground='white',  # 光标颜色
                                                      wrap=tk.WORD)  # 单词换行
        self.output_text.pack(side=tk.RIGHT, padx=10, pady=10)

        self.pcd = []
        self.point_cloud = None
        self.vis = None

        self.load_point_cloud()
        self.start_visualizer_thread()

    def parse_file(self, file_path):
        """
        解析文件并提取点云数据

        :param file_path: 要解析的文件路径
        :return: 包含点云数据的NumPy数组
        """
        points = []
        
        # 打开文件并逐行读取
        with open(file_path, 'r') as file:
            for line in file: 
                # 分割大车位置和JSON部分
                parts = line.strip().split('---')  
                # 如果分割后的部分少于2个，则跳过该行
                if len(parts) < 2:
                    continue
                json_data = parts[1].strip()
                try:
                    # 解析JSON数据
                    json_objects = json.loads(json_data)
                    for obj in json_objects:
                        x = obj['X']
                        y = obj['Y']
                        z = obj['Z']
                        points.append([x, y, z])
                        # 将坐标添加到self.pcd列表中
                        self.pcd.append([x, y, z])
                except json.JSONDecodeError:
                    # 如果解析JSON时出错，输出错误信息
                    self.output_text.insert(tk.END, f"Error decoding JSON: {json_data}\n")
                    self.output_text.see(tk.END)  # 滚动到最新输出
        # 将points列表转换为NumPy数组并返回
        return np.array(points)

    def start_visualizer_thread(self):
        self.visualizer_thread = threading.Thread(target=self.run_visualizer, daemon=True)
        self.visualizer_thread.start()

    def run_visualizer(self):
        """运行可视化器"""
        self.vis = o3d.visualization.VisualizerWithVertexSelection()
        self.vis.create_window("Open3D Visualizer - SHIFT+左键选点 右键清除选点", width=960, height=720)

        # 在 Tkinter 中显示点云
        allcloud = np.array(self.pcd)
        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud.points = o3d.utility.Vector3dVector(allcloud)
        o3d.io.write_point_cloud("./pcd/pointcloud.pcd", self.point_cloud)
        self.vis.add_geometry(self.point_cloud)

        # 获取视图控制器
        view_control = self.vis.get_view_control()
        # 设置视角
        view_control.set_lookat([0, 0, 0])  # 设置相机的目标点
        view_control.set_up([0, 1, 0])      # 设置相机的上方向
        view_control.set_front([0, 0, -1])   # 设置相机的前方向
        view_control.set_zoom(0.5)           # 设置缩放级别

        # 注册选择变化的回调函数
        self.vis.register_selection_changed_callback(self.selection_changed_callback)
        self.vis.register_selection_moved_callback(self.selection_moved_callback)

        # 运行可视化器
        while True:
            try:
                self.vis.update_geometry(self.point_cloud)
                self.vis.poll_events()
                self.vis.update_renderer()
            except Exception as e:
                self.output_text.insert(tk.END, f"Visualizer error: {e}\n")
                break  # 退出循环以防止崩溃

    def selection_moved_callback(self):
        try:
            self.vis.clear_picked_points()
            print("Cleared selected points.")
            self.output_text.insert(tk.END, "Cleared selected points.\n")
            self.output_text.see(tk.END)  # 滚动到最新输出
        except Exception as e:
            print(f"Error clearing selected points: {e}")
            self.output_text.insert(tk.END, f"Error clearing selected points: {e}\n")
            self.output_text.see(tk.END)  # 滚动到最新输出

    def selection_changed_callback(self):
        # 获取当前选择的点
        picked_points = self.vis.get_picked_points()
        if picked_points:
            point_index = picked_points[-1].index
            point_coordinates = np.asarray(self.point_cloud.points)[point_index]
            self.output_text.insert(tk.END, f"Selected ID: {point_index}, coordinate: (x:{point_coordinates[0]}, y:{point_coordinates[1]}, z:{point_coordinates[2]})\n")
            self.output_text.see(tk.END)  # 滚动到最新输出

    def load_point_cloud(self):
        """加载点云数据"""
        file_pattern = os.path.join(self.directory_path, "TestClouds*")
        files = glob.glob(file_pattern)
        files.sort()

        for file in files:
            self.output_text.insert(tk.END, f"Processing file: {file}\n")
            points = self.parse_file(file)
            self.output_text.insert(tk.END, f"Number of points in file {file}: {len(points)}\n")
            if len(points) <= 0:
                continue
        self.output_text.insert(tk.END, f"load {len(files)} files, total pcd size:{len(self.pcd)}\n")
        self.output_text.see(tk.END)  # 滚动到最新输出
        

if __name__ == "__main__":
    # 初始化可视化界面
    root = tk.Tk()
    viewer = PointCloudVisualizer(root, directory_path)
    root.mainloop()
