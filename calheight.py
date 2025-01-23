import open3d as o3d
import numpy as np

TKWIDTH = 0.66
TKHEIGHT = 0.64
TKLENGTH = 1.57
AREA_START = 1.25
AREA_END = 15.55
TKNUMBER = 21
STARTPOS = 13

LOAD_PATH = "./python2dll/dist/pcd/pointcloud_CloudsTest.pcd"
SAVE_PATH = "./python2dll/dist/pcd/filter_CloudsTest.pcd"

def calculate_average_height(point_cloud_file, z_ranges, unit=0.64):
    totalpcd = []
    # 加载点云
    point_cloud = o3d.io.read_point_cloud(point_cloud_file)
    points = np.asarray(point_cloud.points)
    # 删掉所有点云y轴高度大于10的点
    points = points[points[:, 1] < 10]
    start_index = STARTPOS
    # 计算每个范围内的 Z 轴平均高度
    results = []
    # 生成高度表
    height_table = [(i * unit - 0.2, i * unit + 0.2) for i in range(8)]
    # 计算每一列的细分范围
    column_ranges = [(AREA_START + i * TKWIDTH, AREA_START + (i + 1) * TKWIDTH) for i in range(TKNUMBER)]

    for index, (lower, upper) in enumerate(z_ranges):
        # 过滤 Z 轴在范围内的点
        filtered_points = points[(points[:, 2] >= lower) & (points[:, 2] < upper)]
        if filtered_points.size == 0:
            avg_height = None  # 如果没有点在这个范围内
        else:
            for column_lower, column_upper in column_ranges:
                # 过滤在当前列范围内的点
                column_points = filtered_points[(filtered_points[:, 0] >= column_lower) & (filtered_points[:, 0] < column_upper)]
                # 将点云转换为Open3D的PointCloud对象
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(filtered_points)
                totalpcd.extend(filtered_points.tolist())

                # 计算每个高度范围内的点云个数
                point_counts = [0] * len(height_table)
                for point in column_points:
                    for i, (tklower, tkupper) in enumerate(height_table):
                        if tklower <= point[1] <= tkupper:
                            point_counts[i] += 1
                            break

                # 找到点云个数最多的层数
                max_count = max(point_counts)
                max_levels = [i for i, count in enumerate(point_counts) if count == max_count]
                # 计算点云个数最多的层中的平均高度
                avg_heights = []
                for level in max_levels:
                    tklower, tkupper = height_table[level]
                    points_in_level = column_points[(column_points[:, 1] >= tklower) & (column_points[:, 1] < tkupper)]
                    avg_height = np.mean(points_in_level[:, 1]) if points_in_level.size > 0 else None
                    avg_heights.append(avg_height)

                # 记录点云个数最多的层的信息
                if max_levels:
                    level = max_levels[0]  # 只取第一个点云个数最多的层
                    avg_height = avg_heights[0] if avg_heights else None
                    results.append((lower, upper, column_lower, column_upper, level, avg_height, max_count))  # 记录符合条件的结果

    # 将结果按每TKNUMBER个元素为一组，放入一个列表中
    result_groups = []
    for i in range(0, len(results), TKNUMBER):
        result_groups.append(results[i:i+TKNUMBER])
    # 输出结果
    for group_index, group in enumerate(result_groups):
        for lower, upper, column_lower, column_upper, level, avg_height, point_count in group:
            if avg_height is not None:
                print(f"碳块列{start_index} 范围 {lower} - {upper}, 列范围 {column_lower:.2f} - {column_upper:.2f}: 平均高度{avg_height:.2f}为 {level} 层碳块，点云个数为 {point_count}")
            else:
                print(f"碳块列{start_index} 范围 {lower} - {upper}, 列范围 {column_lower:.2f} - {column_upper:.2f}: 平均高度无数据, 为 {level} 层碳块，点云个数为 {point_count}")
        start_index += 1

    # 将结果按二维数组输出
    levellist = []
    for i in range(0, len(results), TKNUMBER):
        levellist.append([result[4] for result in results[i:i+TKNUMBER]])

    # 手动输出formatted_results，每21个元素后换行
    start_index = STARTPOS
    for i in range(0, len(levellist)):
        print(f"{start_index}: ", end=" ")
        for j in range(0, TKNUMBER):
            print(f"{levellist[i][j]:2d}", end=" ")
        print("")
        start_index += 1

    # 保存点云
    savepcd = o3d.geometry.PointCloud()
    savepcd.points = o3d.utility.Vector3dVector(np.array(totalpcd))
    o3d.io.write_point_cloud(SAVE_PATH, savepcd)

# 定义范围数据
right_ranges = [
    44.685, 46.555, 48.425, 50.295, 52.165, 54.985, 56.855, 58.725, 
    60.595, 62.465, 74.813, 76.683, 78.553, 80.423, 82.293, 84.163, 
    86.033, 87.903, 89.773, 91.643, 93.513, 95.383, 97.253, 99.123, 
    114.189, 116.059, 117.929, 119.799, 121.669, 123.539, 125.409, 
    127.279, 129.149, 131.019, 132.889, 134.759, 136.629, 138.499,
    140.369, 142.239, 144.109, 145.979, 147.849, 149.719, 151.589, 153.459
]

left_ranges = [
    43.115, 44.985, 46.855, 48.725, 50.595, 53.415, 55.285, 57.155, 
    59.025, 60.895, 73.243, 75.113, 76.983, 78.853, 80.723, 82.593, 
    84.463, 86.333, 88.203, 90.073, 91.943, 93.813, 95.683, 97.553, 
    112.619, 114.489, 116.359, 118.229, 120.099, 121.969, 123.839,
    125.709, 127.579, 129.449, 131.319, 133.189, 135.059, 136.929,
    138.799, 140.669, 142.539, 144.409, 146.279, 148.149, 150.019, 151.889
]


# 生成 z_ranges
z_ranges = list(zip(left_ranges, right_ranges))
# 调用函数
calculate_average_height(LOAD_PATH, z_ranges)
