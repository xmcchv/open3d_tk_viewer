import threading
import os
import glob
import open3d as o3d
import numpy as np
import configparser
import asyncio
import json
import websockets
import tkinter as tk
from tkinter import scrolledtext
from tkinter import filedialog
from tkinter import messagebox
import ctypes
from datetime import datetime

TKWIDTH = 0.66
TKHEIGHT = 0.64
TKLENGTH = 1.57
AREA_START = 1.25
AREA_END = 15.55
TKNUMBER = 21
STARTPOS = 13

LOAD_PATH = "./python2dll/dist/pcd/pointcloud_CloudsTest.pcd"
SAVE_PATH = "./python2dll/dist/pcd/filter_CloudsTest.pcd"

class Config:
    def __init__(self, config_file="config.ini"):
        self.config = configparser.ConfigParser()
        self.config.read(config_file, encoding='utf-8')
        self.TKWIDTH = float(self.config['CalculateTKHeight']['TKWIDTH'])
        self.TKHEIGHT = float(self.config['CalculateTKHeight']['TKHEIGHT'])
        self.TKLENGTH = float(self.config['CalculateTKHeight']['TKLENGTH'])
        self.AREA_START = float(self.config['CalculateTKHeight']['AREA_START'])
        self.AREA_END = float(self.config['CalculateTKHeight']['AREA_END'])
        self.TKNUMBER = int(self.config['CalculateTKHeight']['TKNUMBER'])
        self.STARTPOS = int(self.config['CalculateTKHeight']['STARTPOS'])
        self.LOAD_PATH = self.config['CalculateTKHeight']['LOAD_PATH']
        self.SAVE_PATH = self.config['CalculateTKHeight']['SAVE_PATH']
        # 生成 z_ranges
        left_ranges = [float(x) for x in self.config['CalculateTKHeight']['left_ranges'].split(',')]
        right_ranges = [float(x) for x in self.config['CalculateTKHeight']['right_ranges'].split(',')]
        self.z_ranges = list(zip(left_ranges, right_ranges))

        self.HOST = self.config['WebSocket']['HOST']
        self.PORT = int(self.config['WebSocket']['PORT'])

        self.FILE_PATTERN = self.config['CalculateTKHeight']['FILE_PATTERN']
        self.DIRECTORY_PATH = self.config['CalculateTKHeight']['DIRECTORY_PATH']
        self.SAVE_JSON_PATH = self.config['CalculateTKHeight']['SAVE_JSON_PATH']

class tkinterApp:
    def __init__(self, config):
        # 获取屏幕的DPI比例
        user32 = ctypes.windll.user32
        user32.SetProcessDPIAware()
        dpi = user32.GetDpiForSystem() / 96.0
        WINDOW_WIDTH = int(640 * dpi)
        WINDOW_HEIGHT = int(480 * dpi)
        self.root = tk.Tk()
        self.root.title("层高计算算法")
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")  # 设置固定窗口大小
        self.root.minsize(int(WINDOW_WIDTH/2), WINDOW_HEIGHT)  # 设置最小窗口大小
        # 创建文本框用于显示输出
        self.output_text = scrolledtext.ScrolledText(self.root, width=600, height=450,
                                                      font=("Courier New", 20),  # 设置字体
                                                      bg="white",  # 设置背景色
                                                      fg="black",  # 设置前景色
                                                      insertbackground='black',  # 光标颜色
                                                      )  # 单词换行
        self.output_text.pack(side=tk.LEFT, padx=10, pady=10)
        self.config = config
        self.pcd = []

class CalculateTKHeight:
    def __init__(self, config):
        self.app = tkinterApp(config)
        self.config = config    # 保存配置信息
        self.totalpcd = []  # 存储所有点云数据
        self.point_cloud = None  # 存储点云数据
        self.results = []
        # 生成高度表
        self.height_table = [(i * self.config.TKHEIGHT - 0.2, i * self.config.TKHEIGHT + 0.2) for i in range(8)]
        # 计算每一列的细分范围
        self.column_ranges = [(self.config.AREA_START + i * self.config.TKWIDTH, self.config.AREA_START + (i + 1) * self.config.TKWIDTH) for i in range(self.config.TKNUMBER)]
        

    def load_point_cloud_from_socket(self, json_data):
        points = []
        # 加载点云
        try:
            # 解析JSON数据
            json_objects = json.loads(json_data)
            for obj in json_objects:
                x = obj['X']
                y = obj['Y']
                z = obj['Z']
                points.append([x, y, z])
            self.point_cloud = o3d.geometry.PointCloud()
            self.point_cloud.points = o3d.utility.Vector3dVector(points)
            self.app.output_text.insert(tk.END, f"receive points: {len(points)}\n")
        except json.JSONDecodeError:
            # 如果解析JSON时出错，输出错误信息
            self.app.output_text.insert(tk.END, f"Error decoding JSON: {json_data}\n")
        self.app.output_text.see(tk.END)  # 滚动到最新输出
        return
    
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
                    json_objects = json.loads(json_data)
                    for obj in json_objects:
                        x = obj['X']
                        y = obj['Y']
                        z = obj['Z']
                        points.append([x, y, z])
                except json.JSONDecodeError:
                    # 如果解析JSON时出错，输出错误信息
                    self.output_text.insert(tk.END, f"Error decoding JSON: {json_data}\n")
                    self.output_text.see(tk.END)  # 滚动到最新输出
        # 将points列表转换为NumPy数组并返回
        return np.array(points)
    
    def load_point_cloud_from_directory(self):
        totalpointclouds = []
        # 遍历目录中的所有文件
        file_pattern = os.path.join(self.config.DIRECTORY_PATH, self.config.FILE_PATTERN)
        self.app.output_text.see(tk.END)  # 滚动到最新输出
        files = glob.glob(file_pattern)
        files.sort()
        total_files = len(files)
        for file in files:
            points = self.parse_file(file)
            if len(points) <= 0:
                continue
            totalpointclouds.extend(points.tolist())
        self.point_cloud = o3d.geometry.PointCloud()
        self.point_cloud.points = o3d.utility.Vector3dVector(totalpointclouds)
        self.app.output_text.insert(tk.END, f"load {total_files} files, total pcd size:{len(totalpointclouds)}, average size:{len(totalpointclouds)/total_files}\n")
        self.app.output_text.see(tk.END)  # 滚动到最新输出
    
    def load_point_cloud_fromPath(self):
        # 加载点云
        self.point_cloud = o3d.io.read_point_cloud(self.config.LOAD_PATH)

    def calculate_average_height(self):
        # 加载点云
        points = np.asarray(self.point_cloud.points)
        # 删掉所有点云y轴高度大于9的点
        points = points[points[:, 1] < 9]
        # 计算每个范围内的 Z 轴平均高度
        self.results = []
        
        for index, (lower, upper) in enumerate(self.config.z_ranges):
            # 过滤 Z 轴在范围内的点
            filtered_points = points[(points[:, 2] >= lower) & (points[:, 2] < upper)]
            if filtered_points.size == 0:
                avg_height = None  # 如果没有点在这个范围内
            else:
                for column_lower, column_upper in self.column_ranges:
                    # 过滤在当前列范围内的点
                    column_points = filtered_points[(filtered_points[:, 0] >= column_lower) & (filtered_points[:, 0] < column_upper)]
                    # 将点云转换为Open3D的PointCloud对象
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(filtered_points)
                    self.totalpcd.extend(filtered_points.tolist())

                    # 计算每个高度范围内的点云个数
                    point_counts = [0] * len(self.height_table)
                    for point in column_points:
                        for i, (tklower, tkupper) in enumerate(self.height_table):
                            if tklower <= point[1] <= tkupper:
                                point_counts[i] += 1
                                break

                    # 找到点云个数最多的层数
                    max_count = max(point_counts)
                    max_levels = [i for i, count in enumerate(point_counts) if count == max_count]
                    # 计算点云个数最多的层中的平均高度
                    avg_heights = []
                    for level in max_levels:
                        tklower, tkupper = self.height_table[level]
                        points_in_level = column_points[(column_points[:, 1] >= tklower) & (column_points[:, 1] < tkupper)]
                        avg_height = np.mean(points_in_level[:, 1]) if points_in_level.size > 0 else None
                        avg_heights.append(avg_height)

                    # 记录点云个数最多的层的信息
                    if max_levels:
                        level = max_levels[0]  # 只取第一个点云个数最多的层
                        avg_height = avg_heights[0] if avg_heights else None
                        self.results.append((lower, upper, column_lower, column_upper, level, avg_height, max_count))  # 记录符合条件的结果
        self.print_results_singletk()

    def save_json_file(self, results_json, file_path):
        """
        将结果保存为JSON文件
        :param results_json: 要保存的JSON数据
        :param file_path: 保存文件的路径
        """
        with open(file_path, 'w') as json_file:
            json.dump(results_json, json_file, indent=4)

    def package_results_json(self):
        results_json = []
        result_groups = []
        for i in range(0, len(self.results), self.config.TKNUMBER):
            result_groups.append(self.results[i:i+self.config.TKNUMBER])
        # 输出结果
        for group_index, group in enumerate(result_groups):
            for row_index, (lower, upper, column_lower, column_upper, level, avg_height, point_count) in enumerate(group):
                if avg_height is not None:
                    result = {
                        "row_index": row_index,
                        "column_index": self.config.STARTPOS + group_index,
                        "level": level,
                    }
                    results_json.append(result)
        # self.app.output_text.insert(tk.END, f"package results: {results_json}\n")
        self.app.output_text.insert(tk.END, f"send results: {len(results_json)}\n")
        self.app.output_text.see(tk.END)  # 滚动到最新输出
        self.save_json_file(results_json, self.config.SAVE_JSON_PATH)
        return results_json    
        
    def print_results(self):
        # 将结果按每TKNUMBER个元素为一组，放入一个列表中
        result_groups = []
        for i in range(0, len(self.results), TKNUMBER):
            result_groups.append(self.results[i:i+TKNUMBER])
        # 输出结果
        for group_index, group in enumerate(result_groups):
            for lower, upper, column_lower, column_upper, level, avg_height, point_count in group:
                if avg_height is not None:
                    print(f"碳块列{start_index} 范围 {lower} - {upper}, 列范围 {column_lower:.2f} - {column_upper:.2f}: 平均高度{avg_height:.2f}为 {level} 层碳块，点云个数为 {point_count}")
                else:
                    print(f"碳块列{start_index} 范围 {lower} - {upper}, 列范围 {column_lower:.2f} - {column_upper:.2f}: 平均高度无数据, 为 {level} 层碳块，点云个数为 {point_count}")
            start_index += 1

    def print_results_singletk(self):
        # 将结果按二维数组输出
        levellist = []
        for i in range(0, len(self.results), self.config.TKNUMBER):
            levellist.append([result[4] for result in self.results[i:i+self.config.TKNUMBER]])

        # 手动输出formatted_results，每21个元素后换行
        start_index = self.config.STARTPOS
        string = []
        for i in range(0, len(levellist)):
            string.append(f"{start_index}: ")
            for j in range(0, self.config.TKNUMBER):
                string.append(f"{levellist[i][j]:2d}")
            string.append("\n")
            start_index += 1
        self.app.output_text.insert(tk.END, "".join(string))
        self.app.output_text.see(tk.END)  # 滚动到最新输出

    def save_point_cloud(self):
        # 保存点云
        savepcd = o3d.geometry.PointCloud()
        savepcd.points = o3d.utility.Vector3dVector(np.array(self.totalpcd))
        o3d.io.write_point_cloud(self.config.SAVE_PATH, savepcd)

class WebSocketServer:
        def __init__(self, config, calculator):
            self.host = config.HOST
            self.port = config.PORT
            self.calculator = calculator
            self.socket_thread = threading.Thread(target=self.run_websocket, daemon=True)
            self.socket_thread.start()

        def run_websocket(self):
            asyncio.run(self.start())

        def decode_json(self, json_data):
            # 解析JSON数据
            json_objects = json.loads(json_data)
            flag = json_objects[0]['flag']
            return flag

        async def handle_client(self, websocket):
            try:
                async for message in websocket:
                    try:
                        self.calculator.app.output_text.insert(tk.END, f"Received JSON data in {datetime.now().strftime('%Y/%m/%d %H:%M:%S')}\n")
                        self.calculator.app.output_text.see(tk.END)  # 滚动到最新输出
                        # self.calculator.load_point_cloud_from_socket(message)
                        flag = self.decode_json(message)
                        if flag == 1:
                            self.calculator.load_point_cloud_from_directory()
                            self.calculator.calculate_average_height()
                        await websocket.send(json.dumps(self.calculator.package_results_json()))
                    except json.JSONDecodeError as e:
                        self.calculator.app.output_text.insert(tk.END, f"Error decoding JSON: {e}\n")
                        self.calculator.app.output_text.see(tk.END)  # 滚动到最新输出
                        await websocket.send(json.dumps({"error": "Invalid JSON format"}))
            except websockets.exceptions.ConnectionClosedError:
                self.calculator.app.output_text.insert(tk.END, f"Client disconnected\n")
                self.calculator.app.output_text.see(tk.END)  # 滚动到最新输出

        async def start(self):
            async with websockets.serve(self.handle_client, self.host, self.port):
                self.calculator.app.output_text.insert(tk.END, f"WebSocket server started at ws://{self.host}:{self.port}\n")
                self.calculator.app.output_text.see(tk.END)  # 滚动到最新输出
                # print(f"WebSocket server started at ws://{self.host}:{self.port}")
                await asyncio.Future()  # 保持服务器运行

if __name__ == "__main__":
    # 加载配置文件
    config = Config("./config.ini")
    # 创建 CalculateTKHeight 实例
    calculator = CalculateTKHeight(config)
    # 创建 WebSocketServer 实例并启动
    ws_server = WebSocketServer(config, calculator)
    # 运行主循环
    calculator.app.root.mainloop()
