# -*- coding: utf-8 -*-

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
import ctypes
import configparser
# import sys
# 设置DPI感知
# ctypes.windll.shcore.SetProcessDpiAwareness(1)

# 获取系统默认编码
# DEFAULT_ENCOING = sys.getfilesystemencoding()

# 获取屏幕的DPI比例
user32 = ctypes.windll.user32
user32.SetProcessDPIAware()
dpi = user32.GetDpiForSystem() / 96.0
WINDOW_WIDTH = int(960 * dpi)
WINDOW_HEIGHT = int(320 * dpi)

POINT_SIZE = 5
FILE_PATTERN = "TestClouds*" 
SAVE_PCD_PATH = "./pcd/pointcloud.pcd"
DIRECTORY_PATH = "C:\\Users\\xmcchv\\Desktop\\宁东\\ndpython\\CloudsTest-----一档跑一列\\CloudsTest"
CONFIG_PATH = "config.ini"

class PointCloudVisualizer:
    def __init__(self, root):
        self.pcd = []
        self.picked_points_list = []
        self.point_cloud = None
        self.visualizer_thread = None
        self.is_running = False  # 控制可视化器是否在运行
        self.is_updated = False  # 控制是否需要更新点云
        self.loading_thread = None
        self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])
        
        self.root = root
        # self.root.tk.call('tk', 'scaling',3.0)
        self.root.title("Point Cloud Visualizer")
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")  # 设置固定窗口大小
        self.root.minsize(int(WINDOW_WIDTH/2), WINDOW_HEIGHT)  # 设置最小窗口大小
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 创建左侧设置框架
        self.settings_frame = tk.Frame(self.root)
        self.settings_frame.pack(side=tk.LEFT, padx=10, pady=8)

        # 在现有的按钮创建代码之后添加以下代码
        # 创建一个框架来放置并排的按钮
        self.info_frame = tk.Frame(self.settings_frame)
        self.info_frame.pack(side=tk.TOP, pady=5)
        # 创建显示info弹窗的按钮，设置字体和大小
        self.info_button = tk.Button(self.info_frame, text="Show Info", font=("微软雅黑", 10), command=self.show_info_popup)
        self.info_button.pack(side=tk.TOP)  # 默认左侧对齐

        self.pattern_frame = tk.Frame(self.settings_frame)
        self.pattern_frame.pack(side=tk.TOP)
        self.progress_label = tk.Label(self.pattern_frame, text="File Pattern:", font=("微软雅黑", 10))
        self.progress_label.pack(side=tk.LEFT,pady=(10, 5))  # 标签与文本框之间的间距
        self.file_pattern_entry = tk.Entry(self.pattern_frame, font=("微软雅黑", 10), width=20)
        self.file_pattern_entry.pack(side=tk.RIGHT,pady=5)

        # 创建文件路径输入框，设置字体和大小
        self.file_path_entry = tk.Entry(self.settings_frame, font=("微软雅黑", 12), width=35)
        self.file_path_entry.pack(pady=5)

        # 创建一个框架来放置并排的按钮
        self.button_frame = tk.Frame(self.settings_frame)
        self.button_frame.pack(pady=5)

        # 创建浏览按钮，设置字体和大小
        self.browse_button = tk.Button(self.button_frame, text="Browse", font=("微软雅黑", 10), command=self.browse_files)
        self.browse_button.pack(side=tk.LEFT, padx=(0, 10))  # 设置右侧间距

        # 创建加载文件按钮，设置字体和大小
        self.load_button = tk.Button(self.button_frame, text="Load Point Cloud", font=("微软雅黑", 10), command=self.start_loading_files)
        self.load_button.pack(side=tk.LEFT)  # 默认左侧对齐

        # 创建进度条及其标签
        self.progress_label = tk.Label(self.settings_frame, text="Progress:", font=("微软雅黑", 10))
        self.progress_label.pack(pady=(10, 5))  # 标签与进度条之间的间距

        self.progress_bar = ttk.Progressbar(self.settings_frame, orient='horizontal', length=350, mode='determinate')
        self.progress_bar.pack(pady=10)

        # 创建可视化按钮，设置字体和大小
        self.visualize_button = tk.Button(self.settings_frame, text="Start Visualizer", font=("微软雅黑", 10), command=self.start_visualizer_thread)
        self.visualize_button.pack(pady=5)

        # 创建保存点云按钮，设置字体和大小
        # 创建一个框架来放置并排的按钮
        self.savepcd_frame = tk.Frame(self.settings_frame)
        self.savepcd_frame.pack(pady=5)
        self.savepcd_path_entry = tk.Entry(self.savepcd_frame, font=("微软雅黑", 12), width=25)
        self.savepcd_path_entry.pack(side=tk.LEFT, padx=5)
        self.savepcd_button = tk.Button(self.savepcd_frame, text="save pcd", font=("微软雅黑", 10), command=lambda: self.save_point_cloud(self.savepcd_path_entry.get(), self.point_cloud))
        self.savepcd_button.pack(side=tk.RIGHT)

        self.output_frame = tk.Frame(self.root)
        self.output_frame.pack(side=tk.RIGHT, padx=10, pady=10)
        # 创建文本框用于显示输出及其标签
        self.output_label = tk.Label(self.output_frame, text="Output:", font=("微软雅黑", 10))
        self.output_label.pack(side=tk.TOP, pady=0)  # 标签与文本框之间的间距
        # 创建文本框显示输出
        self.output_text = scrolledtext.ScrolledText(self.output_frame, width=100, height=20, 
                                                      font=("微软雅黑", 12),  # 设置字体
                                                      bg="white",  # 设置背景色
                                                      fg="black",  # 设置前景色
                                                      insertbackground='white',  # 光标颜色
                                                      wrap=tk.WORD)  # 单词换行
        self.output_text.pack(padx=10, pady=10)

        self.config = configparser.ConfigParser()
        self.config.read(CONFIG_PATH, encoding='utf-8')  # 读取配置文件
        # 初始化文本框的默认值
        self.file_pattern_entry.insert(0, self.config.get('Settings', 'file_pattern', fallback=FILE_PATTERN))
        self.file_path_entry.insert(0, self.config.get('Settings', 'directory_path', fallback=DIRECTORY_PATH))
        self.savepcd_path_entry.insert(0, self.config.get('Settings', 'save_pcd_path', fallback=SAVE_PCD_PATH))
        self.directory_path = DIRECTORY_PATH

    # 定义显示info弹窗的函数
    def show_info_popup(self):
        # 创建一个Toplevel窗口作为弹窗
        info_popup = tk.Toplevel(self.root)
        info_popup.title("About")
        info_popup.geometry("480x320")  # 设置弹窗大小
        info_popup.minsize(320, 160)  # 设置最小窗口大小
        # 设置弹窗显示位置
        # 设置弹窗显示位置
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = (screen_width - 300) // 2
        y = (screen_height - 200) // 2
        info_popup.geometry(f"+{x}+{y}")

        # 在弹窗中添加一个Label来显示信息
        info_text = "1.Browse加载文件夹路径, 单文件选择文件所在文件夹.\n2.load pointcloud解析点云, start visualier启动点云可视化.\n3.可以重新选择路径加载然后解析和显示.\n4.可以使用save_pcd按钮保存点云到指定路径.\n6.正则表达式file_pattern用于设置文件命名的规则.\n"
        info_label = tk.Label(info_popup, text=info_text, font=("微软雅黑", 12), wraplength=450, width=460)
        info_label.pack(padx=5, pady=5, fill=tk.BOTH, expand=True)

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
        # 设置中文文件名，例如："点云数据.pcd"
        directory = os.path.dirname(file_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
            print(f"Directory '{directory}' created.")
        if point_cloud is not None:
            o3d.io.write_point_cloud(file_path, point_cloud)
            self.output_text.insert(tk.END, f"Point cloud saved to {file_path}\n")
            self.output_text.see(tk.END)
        elif len(self.pcd) > 0:
            pointcloud = o3d.geometry.PointCloud()
            pointcloud.points = o3d.utility.Vector3dVector(np.array(self.pcd))
            o3d.io.write_point_cloud(file_path, pointcloud)
            self.output_text.insert(tk.END, f"Point cloud saved to {file_path}\n")
            self.output_text.see(tk.END)
        else:
            self.output_text.insert(tk.END, f"Point clouds is empty, make sure load file first!\n")
            self.output_text.see(tk.END)

    def start_visualizer_thread(self):
        if self.is_running:
            self.is_updated = True  # 如果可视化器正在运行，更新
        else:
            if len(self.pcd) > 0:
                self.is_running = True  # 标记为正在运行
                self.visualizer_thread = threading.Thread(target=self.run_visualizer, daemon=True)
                self.visualizer_thread.start()
            else:
                self.output_text.insert(tk.END, f"Point clouds is empty, make sure load file first!\n")
                self.output_text.see(tk.END)
            # self.root.focus_force()  # 设置Tkinter窗口为焦点窗口

    def run_visualizer(self):
        """运行可视化器"""
        self.vis = o3d.visualization.VisualizerWithVertexSelection()
        self.vis.create_window("Open3D - PointCloudVisualizer", width=960, height=720)
        self.vis.get_render_option().point_size = POINT_SIZE
        self.vis.get_render_option().point_color_option = o3d.visualization.PointColorOption.YCoordinate
        
        self.vis.register_selection_changed_callback(self.selection_changed_callback)
        self.vis.register_selection_moved_callback(self.selection_moved_callback)

        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud.points = o3d.utility.Vector3dVector(np.array(self.pcd))
        self.save_point_cloud(self.savepcd_path_entry.get(), self.point_cloud)
        self.vis.add_geometry(self.point_cloud)

        # 获取视图控制器
        view_control = self.vis.get_view_control()
        points = np.asarray(self.point_cloud.points)  # 获取点云的点坐标
        center = np.mean(points, axis=0)  # 计算中心点
        view_control.set_lookat(center.tolist())  # 设置相机的目标点
        view_control.set_up([0, 1, 0])      # 设置相机的上方向
        view_control.set_front([0, 0, -1])   # 设置相机的前方向
        view_control.set_zoom(0.5)           # 设置缩放级别

        while True:
            if self.is_updated:
                self.is_updated = False
                self.update_pointcloud()
            try:
                self.vis.update_geometry(self.point_cloud)
                self.vis.poll_events()
                self.vis.update_renderer()
                time.sleep(0.03)
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
        time.sleep(0.1)
        picked_points = self.vis.get_picked_points()
        if picked_points:
            picked_points_tuple = []
            for p in picked_points:
                # 确保 p.coord 是一个包含三个元素的列表或数组
                if len(p.coord) == 3:
                    picked_points_tuple.append((p.coord[0], p.coord[1], p.coord[2], p.index))
                else:
                    print(f"Warning: Point {p} has invalid coordinates.")
            new_points = [point for point in picked_points_tuple if tuple(point[:3]) not in self.picked_points_list]
            if new_points:
                for p in new_points:
                    # 获取点的索引和坐标
                    point_index = p[3]
                    point_coordinates = p[:3]
                    self.output_text.insert(tk.END, f"Selected point index: {point_index}, coordinates[ x:{point_coordinates[0]}, y:{point_coordinates[1]}, z:{point_coordinates[2]}]\n")
                    self.output_text.see(tk.END)
                # 将新的选点添加到列表中
                self.picked_points_list.extend(tuple(point[:3]) for point in new_points)
                    
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
            progress_percentage = (i + 1) / len(files) * 100
            self.progress_label.config(text=f"Progress: {progress_percentage:.2f}%")
            # self.update_output_text(f"Processed {i + 1}/{len(files)} files.\n")
            self.root.update_idletasks()  # 刷新 GUI 以更新进度条

        # 文件处理完成后，更新文本框
        self.update_output_text(f"Loaded {len(files)} files, total PCD size: {total_points}, average size: {total_points/len(files) if files else 0 :.4f}\n")
        self.output_text.see(tk.END)  # 滚动到最新输出
        self.progress_label.config(text=f"Progress: Finished!")
        time.sleep(1)
        self.progress_label.config(text=f"Progress:")
        # 清理工作，自动回收线程
        self.loading_thread = None

    def update_output_text(self, message):
        """在主线程中更新文本框"""
        self.output_text.insert(tk.END, message)
        self.output_text.see(tk.END)  # 滚动到最新输出

    def save_config(self):
        """保存配置到文件"""
        print("save config")
        self.config['Settings'] = {
            'file_pattern': self.file_pattern_entry.get(),
            'directory_path': self.file_path_entry.get(),
            'save_pcd_path': self.savepcd_path_entry.get()
        }
        with open(CONFIG_PATH, 'w', encoding='utf-8') as configfile:
            self.config.write(configfile)
    
    def on_closing(self):
        """处理窗口关闭事件"""
        self.save_config()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    viewer = PointCloudVisualizer(root)
    root.mainloop()
