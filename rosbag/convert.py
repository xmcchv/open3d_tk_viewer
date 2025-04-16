import open3d as o3d
import os
import numpy as np

def pcd_to_xyzrgb(input_folder, output_txt_folder, output_npy_folder):
    # 确保输出文件夹存在
    if not os.path.exists(output_txt_folder):
        os.makedirs(output_txt_folder)
    if not os.path.exists(output_npy_folder):
        os.makedirs(output_npy_folder)

    # 遍历指定文件夹中的所有 PCD 文件
    for filename in os.listdir(input_folder):
        if filename.endswith('.pcd'):
            # 构建 PCD 文件的完整路径
            pcd_file_path = os.path.join(input_folder, filename)
            # 读取 PCD 文件
            pcd = o3d.io.read_point_cloud(pcd_file_path)
            
            # 获取点云的坐标和颜色
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)
            # 检查颜色数组是否为空
            if colors.size == 0:
                print(f"Warning: {filename} has no color information. Using default color (white).")
                colors = np.ones_like(points)  # 使用白色作为默认颜色
            # 构建输出文件的名称
            output_txt_path = os.path.join(output_txt_folder, filename.replace('.pcd', '.txt'))
            output_npy_path = os.path.join(output_npy_folder, filename.replace('.pcd', '.npy'))

            # 将点云数据写入 TXT 文件
            with open(output_txt_path, 'w') as f:
                for point, color in zip(points, colors):
                    # 将点的坐标和颜色格式化为一行
                    line = f"{point[0]} {point[1]} {point[2]} {color[0]} {color[1]} {color[2]} 0\n"
                    f.write(line)

            # 保存为 NP格式
            zeros = np.zeros((points.shape[0], 1))  # 创建与点数相同的零数组
            xyzrgb = np.concatenate((points, colors, zeros), axis=1)  # 将点、颜色和零合并
            np.save(output_npy_path, xyzrgb)  # 保存为 .npy 文件

            print(f"Converted {filename} to {output_txt_path} and {output_npy_path}")

# 使用示例
input_folder = './result'  # 替换为你的 PCD 文件夹路径
output_txt_folder = './dataset_txt'  # 替换为输出 TXT 文件夹路径
output_npy_folder = './dataset_npy'  # 替换为输出 NPY 文件夹路径
pcd_to_xyzrgb(input_folder, output_txt_folder, output_npy_folder)
