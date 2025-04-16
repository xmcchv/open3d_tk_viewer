#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sensor_msgs.msg import PointCloud2
import time
import message_filters

SAVE_PATH="/home/ydkj/open3d_tk_viewer/rosbag/result"

def apply_transform(points, transform_matrix):
    """
    应用4x4变换矩阵到点云
    """
    # 转换为齐次坐标
    homogeneous_points = np.hstack([points, np.ones((points.shape[0], 1))])
    transformed_points = homogeneous_points.dot(transform_matrix.T)
    return transformed_points[:, :3]  # 返回前三维坐标

class PointCloudMerger:
    def __init__(self):
        # 初始化变换矩阵（保持原始数值不变）
        self.transforms = {
            '/livox/lidar_3JEDM14001H2691': np.array([
                [-0.384321, -0.921228, -0.0652339, 38.259],
                [-0.9238, 0.38427, 0.0143758, 35.5325],
                [-0.0114883, -0.0656801, 0.99766, 3.43414],
                [0, 0, 0, 1]
            ]),
            '/livox/lidar_3JEDL870019J901': np.array([
                [0.331253, 0.942112, 0.0493408, -5.52502],
                [-0.93201, 0.335793, -0.135247, 35.7111],
                [0.144341, 0.00137704, -0.989415, -3.57784],
                [0, 0, 0, 1]
            ]),
            '/livox/lidar_3JEDL9P001H7181': np.array([
                [-0.731324, -0.682017, -0.00420142, 38.1186],
                [-0.682017, 0.731258, 0.010665, 35.95],
                [0.00420142, -0.010665, 0.999934, -3.204],
                [0, 0, 0, 1]
            ]),
            '/livox/lidar_3JEDK620016G061': np.array([
                [0.211887, 0.976786, 0.00440148, 10.2371],
                [-0.966774, 0.210468, -0.145709, 35.5953],
                [0.14321, -0.0261425, -0.989492, -3.27336],
                [0, 0, 0, 1]
            ])
        }

        # 初始化ROS节点
        rospy.init_node('pointcloud_merger', anonymous=True)
        
        # 创建发布器
        self.pub = rospy.Publisher('/merged_pointcloud', PointCloud2, queue_size=10)
        
        # 创建消息过滤器订阅者
        self.subs = []
        self.topics = [
            '/livox/lidar_3JEDM14001H2691',
            '/livox/lidar_3JEDL870019J901',
            '/livox/lidar_3JEDL9P001H7181',
            '/livox/lidar_3JEDK620016G061'
        ]
        
        for topic in self.topics:
            sub = message_filters.Subscriber(topic, PointCloud2)
            self.subs.append(sub)
        
        # 使用近似时间同步器
        ts = message_filters.ApproximateTimeSynchronizer(self.subs, queue_size=10, slop=0.1)
        ts.registerCallback(self.merge_callback)
        
        rospy.loginfo("Point Cloud Merger Initialized")

    def merge_callback(self, *clouds):
        try:
            combined_points = []
            
            # 遍历每个点云并应用变换
            for cloud, topic in zip(clouds, self.transforms.keys()):
                points = pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
                pc_array = np.array(list(points))
                
                # 应用对应的变换矩阵
                transformed_pc = apply_transform(pc_array, self.transforms[topic])
                combined_points.append(transformed_pc)
            
            # 合并点云
            if combined_points:
                combined_array = np.vstack(combined_points)
                
                # 创建并发布合并后的点云
                header = clouds[0].header
                header.stamp = rospy.Time.now()  # 更新时间戳
                merged_cloud = pc2.create_cloud_xyz32(header, combined_array)
                self.pub.publish(merged_cloud)
                
                # 保存为PCD文件
                self.save_as_pcd(SAVE_PATH, combined_array)
                
        except Exception as e:
            rospy.logerr(f"Error processing point clouds: {str(e)}")

    def save_as_pcd(self, path, points):
        timestamp = int(time.time())
        filename = f"{path}"+f"/combined_{timestamp}.pcd"
        try:
            with open(filename, 'w') as f:
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")
                np.savetxt(f, points, fmt='%.6f')
            rospy.loginfo(f"Saved merged point cloud to {filename}")
        except Exception as e:
            rospy.logerr(f"Failed to save PCD file: {str(e)}")

if __name__ == '__main__':
    try:
        merger = PointCloudMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass