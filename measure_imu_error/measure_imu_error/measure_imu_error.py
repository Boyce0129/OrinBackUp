#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import time

class IMUErrorMeasurementNode(Node):
    def __init__(self):
        super().__init__('measure_imu_error')

        # 參數設置
        self.sample_count = 1000  # 採集 1000 筆數據
        self.sample_frequency = 100  # 假設 IMU 數據頻率為 100Hz
        self.loop_duration = 1.0 / self.sample_frequency  # 每次迴圈 0.01 秒

        # 用於存儲姿態數據
        self.roll_samples = []
        self.pitch_samples = []
        self.yaw_samples = []

        # 訂閱 IMU 數據
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',  # 假設 IMU 數據發布在 /imu/data 話題上
            self.imu_callback,
            10
        )

        # 提示用戶
        self.get_logger().info('Please ensure the drone is stationary on a flat surface.')
        self.get_logger().info('Starting IMU installation error measurement in 2 seconds...')
        time.sleep(2)

        # 開始採集
        self.start_time = time.time()
        self.imu_data_received = False

    def quaternion_to_euler(self, qw, qx, qy, qz):
        """將四元數轉換為歐拉角 (roll, pitch, yaw)，單位：弧度"""
        # 避免除零
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # 處理奇異點
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def imu_callback(self, msg):
        # 確認收到 IMU 數據
        self.imu_data_received = True

        # 提取四元數
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # 將四元數轉換為歐拉角 (roll, pitch, yaw)，單位：弧度
        roll, pitch, yaw = self.quaternion_to_euler(qw, qx, qy, qz)

        # 存儲數據
        self.roll_samples.append(roll)
        self.pitch_samples.append(pitch)
        self.yaw_samples.append(yaw)

        # 檢查是否採集完畢
        if len(self.roll_samples) >= self.sample_count:
            self.calculate_installation_error()
            rclpy.shutdown()  # 採集完成後關閉節點

    def calculate_installation_error(self):
        # 計算平均值
        avg_roll = np.mean(self.roll_samples)
        avg_pitch = np.mean(self.pitch_samples)
        avg_yaw = np.mean(self.yaw_samples)

        # 假設機體坐標系預期姿態為 roll=0, pitch=0, yaw 可作為參考
        expected_roll = 0.0
        expected_pitch = 0.0
        expected_yaw = 0.0  # 可根據實際需求設定

        # 計算安裝誤差角（單位：弧度）
        roll_error = avg_roll - expected_roll
        pitch_error = avg_pitch - expected_pitch
        yaw_error = avg_yaw - expected_yaw

        # 轉換為度數
        rad_to_deg = 180.0 / np.pi
        roll_error_deg = roll_error * rad_to_deg
        pitch_error_deg = pitch_error * rad_to_deg
        yaw_error_deg = yaw_error * rad_to_deg

        # 輸出結果
        self.get_logger().info('IMU Installation Error Measurement Results:')
        self.get_logger().info(f'Roll Error: {roll_error_deg:.4f} degrees')
        self.get_logger().info(f'Pitch Error: {pitch_error_deg:.4f} degrees')
        self.get_logger().info(f'Yaw Error: {yaw_error_deg:.4f} degrees')

        # 計算標準差，評估數據穩定性
        roll_stddev = np.std(self.roll_samples) * rad_to_deg
        pitch_stddev = np.std(self.pitch_samples) * rad_to_deg
        yaw_stddev = np.std(self.yaw_samples) * rad_to_deg

        self.get_logger().info('Measurement Stability (Standard Deviation):')
        self.get_logger().info(f'Roll StdDev: {roll_stddev:.3f} degrees')
        self.get_logger().info(f'Pitch StdDev: {pitch_stddev:.3f} degrees')
        self.get_logger().info(f'Yaw StdDev: {yaw_stddev:.3f} degrees')

        # 提示穩定性
        if roll_stddev > 1.0 or pitch_stddev > 1.0 or yaw_stddev > 1.0:
            self.get_logger().warn('High variance detected. Ensure the drone is stationary and IMU data is stable.')

def main(args=None):
    rclpy.init(args=args)
    node = IMUErrorMeasurementNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()