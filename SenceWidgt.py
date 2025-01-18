import os
import glob
import json
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import time

print("Open3D version:", o3d.__version__)

class PointCloudVisualizer:
    position_info = 0
    pcd = []
    picked_points = []

    def __init__(self):
        # 使用Open3D的SceneWidget
        self.app = gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window("Open3D - PointCloudVisualizer", 1024, 768)

        # 创建SceneWidget
        self.SceneWidget = gui.SceneWidget()
        self.window.add_child(self.SceneWidget)  # 将SceneWidget添加到窗口

        # 创建Open3DScene
        self.renderer = rendering.Open3DScene(self.window.renderer)  # 使用窗口的renderer
        self.SceneWidget.scene = self.renderer  # 将渲染器设置为场景
        
        # self.SceneWidget.set_view_controls(gui.SceneWidget.Controls.PICK_POINTS)
        self.mat = rendering.MaterialRecord()
        self.mat.base_color = [1.0,0.94,0.96,1.0]
        self.mat.shader = "defaultLit"
        self.mat.point_size = 5 * self.window.scaling
        # 初始化点云和坐标轴
        self.point_cloud = None
        self.init_scene()

        # self.SceneWidget.set_view_controls(gui.SceneWidget.Controls.PICK_POINTS)
        # 连接鼠标点击事件
        self.SceneWidget.set_on_mouse(self.on_mouse)

    def init_scene(self):
        # 创建坐标系
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        self.renderer.add_geometry("CoordinateFrame", coordinate_frame, self.mat)

    def parse_file(self, file_path):
        points = []
        
        with open(file_path, 'r') as file:
            for line in file: 
                parts = line.strip().split('---')  # 分割大车位置和JSON部分
                if len(parts) < 2:
                    continue
                
                self.position_info = 360 - float(parts[0].strip())
                print(f"行车位置: {self.position_info}")

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
        
        # 将点云添加到场景
        self.renderer.add_geometry("PointCloud", self.point_cloud, self.mat)

        # 更新视图，设置摄像机位置
        self.update_camera()

    def update_camera(self):
        # 使用Open3DScene来控制相机视角
        target = np.array([0.0, 0.0, self.position_info], dtype=np.float32)  # 相机位置
        eye = np.array([0.0, 0.0, 0.0], dtype=np.float32)               # 目标位置
        up = np.array([0.0, 1.0, 0.0], dtype=np.float32)                   # 上方向
        
        # 使用SceneWidget的look_at方法
        self.SceneWidget.look_at(eye, target, up)

    def on_mouse(self, event: gui.MouseEvent) -> gui.SceneWidget.EventCallbackResult:
        # print(event)
        # print(f'event name:{event.type.name}, value:{event.type.value}')
        # if event.type.value == gui.MouseEvent.BUTTON_DOWN:
        #     print("press down")
        if event.is_modifier_down(gui.KeyModifier.SHIFT) and event.is_button_down(gui.MouseButton.LEFT) and event.type.value == gui.MouseEvent.BUTTON_DOWN:
            print("pick_points")
            mouse_x = event.x
            mouse_y = event.y
            print(f'mouse:({mouse_x},{mouse_y})')
            # # 获取3D坐标
            # picked_point = self.renderer.pick(mouse_x, mouse_y)

            # if picked_point is not None:
            #     print("Picked point coordinates:", picked_point)

            return gui.SceneWidget.EventCallbackResult.HANDLED  # 表示事件已处理
        return gui.SceneWidget.EventCallbackResult.IGNORED  # 事件未处理

    def run(self):
        # 运行窗口
        # self.window.add_child(self.SceneWidget)  # 将SceneWidget添加到窗口
        self.app.run()

if __name__ == "__main__":
    # 初始化可视化界面
    viewer = PointCloudVisualizer()

    # 配置文件路径
    directory_path = "C:\\Users\\xmcchv\\Desktop\\宁东\\ndpython\\CloudsTest--------0116-------2"
    file_pattern = os.path.join(directory_path, "TestClouds*")
    files = glob.glob(file_pattern)
    files.sort()

    # 循环解析文件
    for file in files:
        print(f"Processing file: {file}")
        points = viewer.parse_file(file)
        print(f"Number of points in file {file}: {len(points)}")
        if len(points) <= 0:
            continue

        # 保存点云为PCD文件
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        
        output_file = f"./pcd/{os.path.basename(file)}.pcd"
        o3d.io.write_point_cloud(output_file, point_cloud)
        print(f"Updating point cloud with {len(points)} points.")
    
    # 显示整体点云
    print(f"All point cloud with {len(viewer.pcd)} points.")
    allcloud = np.array(viewer.pcd)
    viewer.visualize_point_cloud(allcloud)

    # 只有在确保点云有效后才写入
    if viewer.point_cloud is not None:
        o3d.io.write_point_cloud("./pcd/pointcloud.pcd", viewer.point_cloud)

    viewer.run()
