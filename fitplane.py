import open3d as o3d
import numpy as np
import copy

"""
    @param file_path 指定点云文件路径
    @param target_height 标定点云平面实际高度

    输出：
    总体 RPY 角度：
    滚转: -90.04°, 俯仰: -0.00°, 偏航: -4.03°
    新增 RPY 角度：
    滚转: -0.04°, 俯仰: -0.00°, 偏航: -4.03°
    新增平移向量(x:0.0, y:0.737760899893015, z:0.0)
"""
file_path = "./pcd/segment.pcd"  # 替换为您的点云文件路径
target_height = 4.48    # 替换为您的目标高度

def load_point_cloud(file_path):
    """加载点云数据"""
    pcd = o3d.io.read_point_cloud(file_path)
    if not pcd.has_points():
        raise ValueError("未能加载点云数据，点云为空。")
    print(f"加载的点云个数: {len(pcd.points)}")  # 打印点云中的点的数量
    return pcd

def compute_normals(pcd):
    """计算点云的法线"""
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    return pcd

def fit_plane(pcd):
    """拟合平面并返回法向量和点"""
    if len(pcd.points) < 3:  # 检查点云中是否有足够的点
        raise RuntimeError("点云中点的数量不足，无法进行平面拟合。")
    
    # 使用 RANSAC 拟合平面
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    normal_vector = plane_model[:3]
    d = plane_model[3]
    return normal_vector, d, inliers

def rotate_to_xoz_plane(pcd, normal_vector):
    """将平面旋转至 X-O-Z 平面"""
    # 目标法线，指向 Y 轴正方向
    target_normal = np.array([0, 1, 0])

    # 计算旋转轴和角度
    rotation_axis = np.cross(normal_vector, target_normal)
    rotation_angle = np.arccos(np.dot(normal_vector, target_normal) / (np.linalg.norm(normal_vector) * np.linalg.norm(target_normal)))

    # 如果旋转轴为零向量，说明已经对齐，无需旋转
    if np.linalg.norm(rotation_axis) > 1e-6:
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)  # 单位化

        # 生成旋转矩阵
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * rotation_angle)
        pcd.rotate(rotation_matrix, center=(0, 0, 0))  # 以原点为中心旋转

        print("计算得到的旋转矩阵：\n", rotation_matrix)
        # 从滚转、俯仰、偏航角构造旋转矩阵（示例）
        r, p, y = -90, 0, 0  # 示例的滚转、俯仰、偏航角
        euler_rotation_matrix = get_rotation_matrix_from_euler(r, p, y)
        print("从 RPY 角度构造的旋转矩阵：\n", euler_rotation_matrix)

        # 右乘旋转矩阵的逆矩阵
        combined_rotation = rotation_matrix @ euler_rotation_matrix
        print("整体变换矩阵：\n", combined_rotation)

        # 将旋转矩阵转换为 RPY
        roll, pitch, yaw = rotation_matrix_to_rpy(combined_rotation)
        print(f"总体 RPY 角度：\n滚转: {roll:.2f}°, 俯仰: {pitch:.2f}°, 偏航: {yaw:.2f}°")
        roll, pitch, yaw = rotation_matrix_to_rpy(rotation_matrix)
        print(f"新增 RPY 角度：\n滚转: {roll:.2f}°, 俯仰: {pitch:.2f}°, 偏航: {yaw:.2f}°")

        # 计算点云的中心
        center = np.mean(np.asarray(pcd.points), axis=0)

        # 计算平移向量
        current_y = center[1]
        translation_y = target_height - current_y  # 计算需要的平移量
        translation_vector = np.array([0, translation_y, 0])  # 只在 Y 轴方向上平移
        # 应用平移
        print(f"新增平移向量(x:{translation_vector[0]}, y:{translation_vector[1]}, z:{translation_vector[2]})")


def rotation_matrix_to_rpy(rotation_matrix):
    """将旋转矩阵转换为滚转、俯仰、偏航角度"""
    # 确保旋转矩阵是有效的
    if np.abs(rotation_matrix[2, 0]) < 1:  # 检查是否存在万向锁
        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = np.arcsin(-rotation_matrix[2, 0])
        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        # 万向锁情况
        yaw = 0
        if rotation_matrix[2, 0] > 0:
            pitch = np.pi / 2
            roll = np.arctan2(rotation_matrix[0, 1], rotation_matrix[0, 2])
        else:
            pitch = -np.pi / 2
            roll = np.arctan2(-rotation_matrix[0, 1], -rotation_matrix[0, 2])

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)  # 转换为度

def draw_point_clouds(original_pcd, rotated_pcd):
    """可视化原始点云和旋转后的点云，显示坐标轴"""
    # 创建坐标轴
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])

    # 将原始点云和旋转后的点云合并到同一个几何体列表中
    geometries = [original_pcd.paint_uniform_color([1, 0, 0]),  # 红色显示原始点云
                  rotated_pcd.paint_uniform_color([0, 1, 0]),   # 绿色显示旋转后的点云
                  axis]  # 添加坐标轴

    # 在同一个窗口中可视化
    o3d.visualization.draw_geometries(geometries, window_name="Point Clouds and Coordinate Frame")

def main(file_path):
    # 1. 加载点云
    pcd = load_point_cloud(file_path)

    # 2. 计算法线
    pcd = compute_normals(pcd)

    # 3. 拟合平面
    normal_vector, d, inliers = fit_plane(pcd)

    # 4. 将拟合的平面旋转至 X-O-Z 平面
    # 保存旋转前的点云以便可视化
    original_pcd = copy.deepcopy(pcd)
    rotate_to_xoz_plane(pcd, normal_vector)

    # 5. 可视化旋转前后的点云和坐标轴
    draw_point_clouds(original_pcd, pcd)

def get_rotation_matrix_from_euler(r, p, y):
    """从滚转、俯仰、偏航角计算旋转矩阵"""
    # 将角度转换为弧度
    r, p, y = np.radians([r, p, y])

    # 计算旋转矩阵
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(r), -np.sin(r)],
                    [0, np.sin(r), np.cos(r)]])
    
    R_y = np.array([[np.cos(p), 0, np.sin(p)],
                    [0, 1, 0],
                    [-np.sin(p), 0, np.cos(p)]])
    
    R_z = np.array([[np.cos(y), -np.sin(y), 0],
                    [np.sin(y), np.cos(y), 0],
                    [0, 0, 1]])

    # 总旋转矩阵
    R = R_z @ R_y @ R_x  # 先绕 x 旋转，再绕 y 旋转，最后绕 z 旋转
    return R


if __name__ == "__main__":
    main(file_path)
