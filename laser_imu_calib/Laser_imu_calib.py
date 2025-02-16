#!/usr/bin/env python3
import rospy
import numpy as np
from scipy.optimize import minimize
from geometry_msgs.msg import TransformStamped, TwistStamped
from sensor_msgs.msg import Imu, LaserScan
import message_filters
import tf.transformations as tf_trans

class LidarImuCalibrator:
    def __init__(self):
        # 初始化参数：外参矩阵 (旋转向量和平移)
        self.extrinsic_r = np.zeros(3)  # 旋转向量（轴角表示）
        self.extrinsic_t = np.zeros(3)  # 平移向量

        # 存储同步后的IMU和雷达位姿数据对
        self.data_pairs = []

        # ROS订阅
        imu_sub = message_filters.Subscriber("/imu/data", Imu)
        lidar_sub = message_filters.Subscriber("/scan", LaserScan)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [imu_sub, lidar_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        # IMU积分状态变量
        self.last_imu_time = None
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.current_orientation = np.eye(3)  # 初始姿态为无旋转
        self.gravity = np.array([0, 0, -9.81])  # 重力补偿（Z轴向下）

        # 雷达扫描匹配状态变量
        self.last_scan_points = None  # 保存上一帧点云

        # IMU标定参数
        self.gyr_params = {
            "unit": "rad/s",
            "avg-axis": {"gyr_n": 1.2630104152196798e-03, "gyr_w": 8.5380643678558898e-05},
            "x-axis": {"gyr_n": 1.0352134048746743e-03, "gyr_w": 6.9942025227676870e-05},
            "y-axis": {"gyr_n": 1.8319581105949590e-03, "gyr_w": 1.2455275772566552e-04},
            "z-axis": {"gyr_n": 9.2185973018940603e-04, "gyr_w": 6.1647148082334360e-05}
        }
        self.acc_params = {
            "unit": "m/s^2",
            "avg-axis": {"acc_n": 3.3056570880789470e-02, "acc_w": 2.3174698409400771e-04},
            "x-axis": {"acc_n": 1.9326644473729518e-02, "acc_w": 2.2565548565140286e-04},
            "y-axis": {"acc_n": 5.0914430175137672e-02, "acc_w": 2.9830400055206625e-04},
            "z-axis": {"acc_n": 2.8928637993501205e-02, "acc_w": 1.7128146607855399e-04}
        }

    def sync_callback(self, imu_msg, lidar_msg):
        # 步骤1：从IMU积分获取当前位姿（简化示例，实际需积分处理）
        T_imu = self.integrate_imu(imu_msg)

        # 步骤2：从雷达数据估计位姿（需实现scan matching）
        T_lidar = self.estimate_lidar_pose(lidar_msg)

        # 存储数据对
        self.data_pairs.append((T_imu, T_lidar))

    def extrinsic_to_matrix(self, rvec, tvec):
        """将旋转向量和平移转换为4x4变换矩阵"""
        R = tf_trans.rotation_matrix(rvec)
        T = np.eye(4)
        T[:3, :3] = R[:3, :3]
        T[:3, 3] = tvec
        return T

    def cost_function(self, params):
        """计算外参优化损失"""
        rvec, tvec = params[:3], params[3:]
        T_LI = self.extrinsic_to_matrix(rvec, tvec)
        total_error = 0.0

        for T_imu, T_lidar in self.data_pairs:
            # 理论雷达位姿：T_LI * T_imu * T_LI^{-1}（根据标定模型）
            # 简化计算（需根据实际模型调整）
            T_pred = np.dot(T_LI, T_imu)
            error = np.linalg.norm(T_pred[:3, 3] - T_lidar[:3, 3])  # 平移误差
            total_error += error

        return total_error / len(self.data_pairs)

    def calibrate(self):
        """启动优化"""
        initial_params = np.concatenate([self.extrinsic_r, self.extrinsic_t])
        result = minimize(self.cost_function, initial_params, method='BFGS')
        optimized_params = result.x
        self.extrinsic_r = optimized_params[:3]
        self.extrinsic_t = optimized_params[3:]

        # 输出结果
        print("Optimized Extrinsic:")
        print("Rotation Vector:", self.extrinsic_r)
        print("Translation:", self.extrinsic_t)

    # ----------- IMU积分实现 ----------- 
    def integrate_imu(self, imu_msg):
        """从IMU数据积分计算位姿（简化版）"""
        if self.last_imu_time is None:
            self.last_imu_time = imu_msg.header.stamp.to_sec()
            return np.eye(4)  # 初始位姿

        current_time = imu_msg.header.stamp.to_sec()
        dt = current_time - self.last_imu_time
        if dt <= 0:
            return np.eye(4)

        # 提取角速度和线加速度
        angular_vel = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])
        linear_acc = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])

        # 姿态更新：使用旋转矢量积分
        angle = np.linalg.norm(angular_vel) * dt
        if angle > 1e-6:
            axis = angular_vel / np.linalg.norm(angular_vel)
            delta_rot = tf_trans.rotation_matrix(angle, axis)[:3, :3]
            self.current_orientation = np.dot(self.current_orientation, delta_rot)

        # 加速度转换到世界坐标系并补偿重力
        acc_world = np.dot(self.current_orientation, linear_acc) - self.gravity

        # 速度积分（中值积分简化）
        self.current_velocity += acc_world * dt

        # 位置积分
        self.current_position += self.current_velocity * dt + 0.5 * acc_world * dt**2

        # 更新时间
        self.last_imu_time = current_time

        # 构建位姿矩阵
        T = np.eye(4)
        T[:3, :3] = self.current_orientation
        T[:3, 3] = self.current_position
        return T

    # ----------- 雷达位姿估计实现 ----------- 
    def estimate_lidar_pose(self, lidar_msg):
        """基于ICP的雷达位姿估计（2D简化版）"""
        # 转换为二维点云
        ranges = np.array(lidar_msg.ranges)
        angle_min = lidar_msg.angle_min
        angle_increment = lidar_msg.angle_increment

        # 过滤无效数据
        valid = (ranges >= lidar_msg.range_min) & (ranges <= lidar_msg.range_max)
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        valid_angles = angles[valid]
        valid_ranges = ranges[valid]

        # 转换为笛卡尔坐标
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        current_points = np.vstack((x, y)).T  # Nx2

        if self.last_scan_points is None or len(self.last_scan_points) < 10:
            self.last_scan_points = current_points
            return np.eye(4)  # 第一帧无法计算

        # 使用简化ICP估计相对位姿
        transform = self.icp_2d(self.last_scan_points, current_points, max_iterations=20)
        self.last_scan_points = current_points  # 更新上一帧

        # 转换为4x4矩阵（假设2D平面运动）
        T = np.eye(4)
        T[:2, :2] = transform[:2, :2]
        T[:2, 3] = transform[:2, 2]
        return T

    def icp_2d(self, src, dst, max_iterations=20, tolerance=1e-5):
        """2D ICP实现（返回3x3齐次变换矩阵）"""
        transform = np.eye(3)  # 初始猜测
        prev_error = 0

        for _ in range(max_iterations):
            # 变换源点
            src_hom = np.hstack((src, np.ones((src.shape[0], 1))))
            transformed_src = np.dot(src_hom, transform.T)[:, :2]

            # 最近邻匹配
            indices = self.nearest_neighbor(transformed_src, dst)
            corresponding_dst = dst[indices]

            # 计算最优变换
            transform = self.umeyama_algorithm(transformed_src, corresponding_dst)

            # 检查收敛
            error = np.mean(np.linalg.norm(transformed_src - corresponding_dst, axis=1))
            if np.abs(prev_error - error) < tolerance:
                break
            prev_error = error

        return transform

    def nearest_neighbor(self, src, dst):
        """最近邻搜索（暴力法）"""
        indices = np.zeros(src.shape[0], dtype=int)
        for i in range(src.shape[0]):
            distances = np.linalg.norm(dst - src[i], axis=1)
            indices[i] = np.argmin(distances)
        return indices

    def umeyama_algorithm(self, src, dst):
        """Umeyama最小二乘对齐（返回3x3齐次变换矩阵）"""
        src_mean = np.mean(src, axis=0)
        dst_mean = np.mean(dst, axis=0)

        src_centered = src - src_mean
        dst_centered = dst - dst_mean

        H = np.dot(src_centered.T, dst_centered)
        U, _, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        # 避免反射
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)

        t = dst_mean - np.dot(R, src_mean)

        transform = np.eye(3)
        transform[:2, :2] = R
        transform[:2, 2] = t
        return transform

if __name__ == "__main__":
    rospy.init_node("lidar_imu_calib")
    calibrator = LidarImuCalibrator()
    rospy.sleep(10)  # 等待数据采集
    calibrator.calibrate()
