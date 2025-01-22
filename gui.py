import os
import glob
import json
import numpy as np
import open3d as o3d
import tkinter as tk
from tkinter import scrolledtext
from tkinter import filedialog
from tkinter import ttk  # 导入ttk模块
import threading
import time

POINT_SIZE = 5
FILE_PATTERN = "TestClouds*" 
SAVE_PCD_PATH = "./pcd/pointcloud.pcd"
DIRECTORY_PATH = "C:\\Users\\xmcchv\\Desktop\\宁东\\ndpython\\CloudsTest27"

class PointCloudVisualizer:
    def __init__(self, root, directory_path):
        self.root = root
        self.root.title("Point Cloud Visualizer")
        self.root.geometry("960x320")  # 设置固定窗口大小
        self.root.minsize(960, 320)  # 设置最小窗口大小
        self.directory_path = directory_path
        
        # 创建左侧设置框架
        self.settings_frame = tk.Frame(self.root)
        self.settings_frame.pack(side=tk.LEFT, padx=10, pady=10)

        self.pattern_frame = tk.Frame(self.settings_frame)
        self.pattern_frame.pack(side=tk.TOP)
        self.progress_label = tk.Label(self.pattern_frame, text="File Pattern:", font=("Courier New", 10))
        self.progress_label.pack(side=tk.LEFT,pady=(10, 5))  # 标签与文本框之间的间距
        self.file_pattern_entry = tk.Entry(self.pattern_frame, font=("Courier New", 10), width=20)
        self.file_pattern_entry.pack(side=tk.RIGHT,pady=5)
        self.file_pattern_entry.insert(0, FILE_PATTERN)  # 插入默认路径

        # 创建文件路径输入框，设置字体和大小
        self.file_path_entry = tk.Entry(self.settings_frame, font=("Courier New", 10), width=35)
        self.file_path_entry.pack(pady=5)
        self.file_path_entry.insert(0, directory_path)  # 插入默认路径

        # 创建一个框架来放置并排的按钮
        self.button_frame = tk.Frame(self.settings_frame)
        self.button_frame.pack(pady=5)

        # 创建浏览按钮，设置字体和大小
        self.browse_button = tk.Button(self.button_frame, text="Browse", font=("Courier New", 10), command=self.browse_files)
        self.browse_button.pack(side=tk.LEFT, padx=(0, 10))  # 设置右侧间距

        # 创建加载文件按钮，设置字体和大小
        self.load_button = tk.Button(self.button_frame, text="Load Point Cloud", font=("Courier New", 10), command=self.start_loading_files)
        self.load_button.pack(side=tk.LEFT)  # 默认左侧对齐

        # 创建进度条及其标签
        self.progress_label = tk.Label(self.settings_frame, text="Progress:", font=("Courier New", 10))
        self.progress_label.pack(pady=(10, 5))  # 标签与进度条之间的间距

        self.progress_bar = ttk.Progressbar(self.settings_frame, orient='horizontal', length=300, mode='determinate')
        self.progress_bar.pack(pady=10)

        # 创建可视化按钮，设置字体和大小
        self.visualize_button = tk.Button(self.settings_frame, text="Start Visualizer", font=("Courier New", 10), command=self.start_visualizer_thread)
        self.visualize_button.pack(pady=5)


        self.output_frame = tk.Frame(self.root)
        self.output_frame.pack(side=tk.RIGHT, padx=10, pady=10)
        # 创建文本框用于显示输出及其标签
        self.output_label = tk.Label(self.output_frame, text="Output:", font=("Courier New", 10))
        self.output_label.pack(pady=0)  # 标签与文本框之间的间距
        # 创建文本框显示输出
        self.output_text = scrolledtext.ScrolledText(self.output_frame, width=100, height=20, 
                                                      font=("Courier New", 15),  # 设置字体
                                                      bg="white",  # 设置背景色
                                                      fg="black",  # 设置前景色
                                                      insertbackground='white',  # 光标颜色
                                                      wrap=tk.WORD)  # 单词换行
        self.output_text.pack(padx=10, pady=10)

        self.pcd = []
        self.point_cloud = None
        self.visualizer_thread = None
        self.is_running = False  # 控制可视化器是否在运行
        self.is_updated = False  # 控制是否需要更新点云
        self.loading_thread = None
        self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])

    def browse_files(self):
        directory_path = filedialog.askdirectory()
        if directory_path:
            self.file_path_entry.delete(0, tk.END)
            self.file_path_entry.insert(0, directory_path)

    def parse_file(self, file_path):
        points = []
        with open(file_path, 'r') as file:
            for line in file: 
                parts = line.strip().split('---')
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
                    self.output_text.see(tk.END)
        return np.array(points)

    def save_point_cloud(self, file_path, point_cloud):
        """保存点云数据"""
        directory = os.path.dirname(file_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
            print(f"Directory '{directory}' created.")
        if point_cloud is not None:
            o3d.io.write_point_cloud(file_path, point_cloud)
            self.output_text.insert(tk.END, f"Point cloud saved to {file_path}\n")

    def start_visualizer_thread(self):
        if self.is_running:
            self.is_updated = True  # 如果可视化器正在运行，更新
        else:
            self.is_running = True  # 标记为正在运行
            self.visualizer_thread = threading.Thread(target=self.run_visualizer, daemon=True)
            self.visualizer_thread.start()

    def run_visualizer(self):
        """运行可视化器"""
        self.vis = o3d.visualization.VisualizerWithVertexSelection()
        self.vis.create_window("Open3D - PointCloudVisualizer", width=960, height=720)
        
        self.vis.register_selection_changed_callback(self.selection_changed_callback)
        self.vis.register_selection_moved_callback(self.selection_moved_callback)

        allcloud = np.array(self.pcd)
        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud.points = o3d.utility.Vector3dVector(allcloud)
        self.save_point_cloud(SAVE_PCD_PATH, self.point_cloud)
        
        self.vis.add_geometry(self.point_cloud)

        while True:
            if self.is_updated:
                self.is_updated = False
                self.update_pointcloud()
            try:
                self.vis.update_geometry(self.point_cloud)
                self.vis.poll_events()
                self.vis.update_renderer()
                time.sleep(0.04)
            except Exception as e:
                self.output_text.insert(tk.END, f"Visualizer error: {e}\n")
                break
        self.vis.close()  # 关闭可视化器窗口
        self.visualizer_thread = None
        self.is_running = False  # 标记为不再运行
    
    def update_pointcloud(self):
        self.point_cloud.points = o3d.utility.Vector3dVector(np.array(self.pcd))  # 确保更新点云
        self.vis.clear_geometries()
        self.vis.add_geometry(self.point_cloud)
        self.output_text.insert(tk.END, f"reload pointcloud to {len(self.pcd)}.\n")
        self.output_text.see(tk.END)

    def selection_moved_callback(self):
        self.vis.clear_picked_points()

    def selection_changed_callback(self):
        picked_points = self.vis.get_picked_points()
        if picked_points:
            # 获取最后一个点的索引
            last_point = picked_points[-1]
            point_index = last_point.index  # 获取点的索引
            points_array = np.asarray(self.point_cloud.points)
            # 确保 point_index 是有效的索引
            if 0 <= point_index < len(points_array):
                point_coordinates = points_array[point_index]  # 获取点的坐标
                self.output_text.insert(tk.END, f"Selected point index: {point_index}, coordinates[ x:{point_coordinates[0]}, y:{point_coordinates[1]}, z:{point_coordinates[2]}]\n")
                self.output_text.see(tk.END)

    def start_loading_files(self):
        """启动线程加载文件"""
        self.output_text.delete(1.0, tk.END)  # 清空输出文本框
        directory_path = self.file_path_entry.get()
        self.loading_thread = threading.Thread(target=self.load_point_cloud, args=(directory_path,), daemon=True)
        self.loading_thread.start()

    def load_point_cloud(self, directory_path):
        """加载点云数据"""
        self.pcd = []
        file_pattern = os.path.join(directory_path, self.file_pattern_entry.get())
        files = glob.glob(file_pattern)
        files.sort()
        self.progress_bar['maximum'] = len(files)  # 设置进度条最大值
        total_points = 0

        for i, file in enumerate(files):
            self.update_output_text(f"Processing file: {file}\n")
            points = self.parse_file(file)
            total_points += len(points)
            self.update_output_text(f"Number of points in file {os.path.basename(file)}: {len(points)}\n")

            # 更新进度
            self.progress_bar['value'] = i + 1  # 更新进度条的当前值
            # self.update_output_text(f"Processed {i + 1}/{len(files)} files.\n")
            self.root.update_idletasks()  # 刷新 GUI 以更新进度条

        # 文件处理完成
        self.update_output_text(f"Loaded {len(files)} files, total PCD size: {total_points}, average size: {total_points/len(files) if files else 0}\n")
        self.output_text.see(tk.END)  # 滚动到最新输出
        # 清理工作，自动回收线程
        self.loading_thread = None

    def update_output_text(self, message):
        """在主线程中更新文本框"""
        self.output_text.insert(tk.END, message)
        self.output_text.see(tk.END)  # 滚动到最新输出

if __name__ == "__main__":
    root = tk.Tk()
    viewer = PointCloudVisualizer(root, DIRECTORY_PATH)
    root.mainloop()
