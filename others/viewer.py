import os
import glob
import json
import numpy as np
import open3d as o3d
import tkinter as tk
from tkinter import scrolledtext
from tkinter import filedialog
import threading
import sys
print(sys.version)

"""
    open3d点云可视化  
    环境安装: pip install open3d numpy
    :=param DIRECTORY_PATH 配置文件路径
    :=param ISFILE 是否是文件 单个文件True 文件夹False
    :=param FILE_PATTERN 文件相同的部分,读取文件夹时设置 如："TestClouds*"
    :=param SAVE_PCD_PATH 保存点云的路径
    :=param POINT_SIZE 点的大小
    使用方式： SHIFT+左键选点 右键清除选点
"""
POINT_SIZE = 3
ISFILE= False 
FILE_PATTERN = "TestClouds*" 
SAVE_PCD_PATH = "./pcd/pointcloud.pcd"
BASE_PATH = "C:\\Users\\xmcchv\\Desktop\\宁东\\ndpython\\"

# DIRECTORY_PATH = "CloudsTest-----一档跑一列\CloudsTest"
# DIRECTORY_PATH = "CloudsTest单雷达由东到西---东雷达61-12列\CloudsTest"
# DIRECTORY_PATH = "CloudsTest------e--0122----01\CloudsTest"
# DIRECTORY_PATH = "CloudsTest单雷达由东到西---东雷达61-12列\\CloudsTest"
# DIRECTORY_PATH = "CloudsTest------17-13\\CloudsTest"
DIRECTORY_PATH = "CloudsTest-----17-13-1\\CloudsTest"
# DIRECTORY_PATH = "CloudsTest单雷达由东到西---东雷达61-12列\\CloudsTest"



# 12 -> 61  两个雷达   61-> 12 由东向西 东雷达

# 10.12 - 10.15
# 自西向东
# DIRECTORY_PATH = "C:\\Users\\xmcchv\\Desktop\\宁东\\ndpython\\CloudsTest------250120-------1"
# 10.01 - 10.04
# 自东向西
# DIRECTORY_PATH = "C:\\Users\\xmcchv\\Desktop\\宁东\\ndpython\\CloudsTest--------0120--------2"

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
                        # if(z > 153.459 or z < 43.115):
                        #     continue
                        # angv = obj['AngV']
                        # if angv != 14.98:
                        #     continue
                        points.append([x, y, z])
                        # 将坐标添加到self.pcd列表中
                        self.pcd.append([x, y, z])
                except json.JSONDecodeError:
                    # 如果解析JSON时出错，输出错误信息
                    self.output_text.insert(tk.END, f"Error decoding JSON: {json_data}\n")
                    self.output_text.see(tk.END)  # 滚动到最新输出
        # 将points列表转换为NumPy数组并返回
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
        """
        启动可视化器线程。
        此方法创建一个新线程，该线程将运行 `run_visualizer` 方法，以在后台处理点云数据的可视化。
        """
        # 创建一个新的线程，目标函数是 self.run_visualizer，设置为守护线程
        self.visualizer_thread = threading.Thread(target=self.run_visualizer, daemon=True)
        # 启动线程
        self.visualizer_thread.start()

    def run_visualizer(self):
        """运行可视化器"""
        global POINT_SIZE
        self.vis = o3d.visualization.VisualizerWithVertexSelection()
        self.vis.create_window("Open3D Visualizer - SHIFT+左键选点 右键清除选点", width=960, height=720)

        # 在 Tkinter 中显示点云
        allcloud = np.array(self.pcd)
        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud.points = o3d.utility.Vector3dVector(allcloud)
        self.save_point_cloud(SAVE_PCD_PATH, self.point_cloud)
        self.vis.add_geometry(self.point_cloud)

        self.vis.get_render_option().point_size = POINT_SIZE
        self.vis.get_render_option().point_color_option = o3d.visualization.PointColorOption.YCoordinate
        # 获取视图控制器
        view_control = self.vis.get_view_control()
        # 设置视角
        # 计算点云的中心
        points = np.asarray(self.point_cloud.points)  # 获取点云的点坐标
        center = np.mean(points, axis=0)  # 计算中心点
        view_control.set_lookat(center.tolist())  # 设置相机的目标点
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
        if picked_points is not None:
            point_index = picked_points[-1].index
            point_coordinates = np.asarray(self.point_cloud.points)[point_index]
            self.output_text.insert(tk.END, f"Selected ID: {point_index}, coordinate: (x:{point_coordinates[0]}, y:{point_coordinates[1]}, z:{point_coordinates[2]})\n")
            self.output_text.see(tk.END)  # 滚动到最新输出

    def load_point_cloud(self):
        """加载点云数据,根据ISFILE判断是文件路径还是文件夹路径"""
        if ISFILE:
            self.output_text.insert(tk.END, f"Processing file: {file}\n")
            points = self.parse_file(self.directory_path)
            self.output_text.insert(tk.END, f"Number of points in file {os.path.basename(file)}: {len(points)}\n")
            if len(points) <= 0:
                self.output_text.insert(tk.END, f"file {os.path.basename(file)} is empty, please make sure it has points!\n")
            self.output_text.see(tk.END)  # 滚动到最新输出
        else:
            file_pattern = os.path.join(self.directory_path, FILE_PATTERN)
            files = glob.glob(file_pattern)
            files.sort()

            for file in files:
                self.output_text.insert(tk.END, f"Processing file: {file}\n")
                points = self.parse_file(file)
                self.output_text.insert(tk.END, f"Number of points in file {os.path.basename(file)}: {len(points)}\n")
                if len(points) <= 0:
                    continue
            self.output_text.insert(tk.END, f"load {len(files)} files, total pcd size:{len(self.pcd)}, average size:{len(self.pcd)/len(files)}\n")
            self.output_text.see(tk.END)  # 滚动到最新输出
        

if __name__ == "__main__":
    # 初始化可视化界面
    root = tk.Tk()
    viewer = PointCloudVisualizer(root, BASE_PATH + DIRECTORY_PATH)
    root.mainloop()
