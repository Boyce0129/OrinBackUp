#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

import time
import smbus
from ctypes import c_short
import math
from collections import deque

# ========== I2C / Sensor 基本設定 ==========
DEVICE = 0x76           # BME/BMP280 常見位址 0x76 或 0x77
bus = smbus.SMBus(1)    # 標準在 Raspberry Pi/Ubuntu 上通常是 1

# ======= 小工具：把 bytes 轉成有號/無號數值 =======
def getShort(data, index):
    return c_short((data[index+1] << 8) + data[index]).value

def getUShort(data, index):
    return (data[index+1] << 8) + data[index]

def getChar(data, index):
    result = data[index]
    if result > 127:
        result -= 256
    return result

def getUChar(data, index):
    return data[index] & 0xFF

# ======= 讀取 BME280 / BMP280 感測值（溫度/壓力/溼度）=======
def readBME280All(addr=DEVICE):
    REG_DATA     = 0xF7
    REG_CONTROL  = 0xF4
    REG_CONFIG   = 0xF5
    REG_CONTROL_HUM = 0xF2

    OVERSAMPLE_TEMP = 2
    OVERSAMPLE_PRES = 2
    OVERSAMPLE_HUM  = 2   # 若為 BMP280（無溼度），設定不影響
    MODE = 1              # Forced mode

    # 設定濕度 oversample（BME280 有效；BMP280 無此暫存器 -> 忽略）
    try:
        bus.write_byte_data(addr, REG_CONTROL_HUM, OVERSAMPLE_HUM)
    except Exception:
        pass

    control = (OVERSAMPLE_TEMP << 5) | (OVERSAMPLE_PRES << 2) | MODE
    bus.write_byte_data(addr, REG_CONTROL, control)

    # 讀取校正值
    cal1 = bus.read_i2c_block_data(addr, 0x88, 24)
    cal2 = bus.read_i2c_block_data(addr, 0xA1, 1)
    cal3 = bus.read_i2c_block_data(addr, 0xE1, 7)

    # 溫度校正
    dig_T1 = getUShort(cal1, 0)
    dig_T2 = getShort(cal1, 2)
    dig_T3 = getShort(cal1, 4)

    # 壓力校正
    dig_P1 = getUShort(cal1, 6)
    dig_P2 = getShort(cal1, 8)
    dig_P3 = getShort(cal1, 10)
    dig_P4 = getShort(cal1, 12)
    dig_P5 = getShort(cal1, 14)
    dig_P6 = getShort(cal1, 16)
    dig_P7 = getShort(cal1, 18)
    dig_P8 = getShort(cal1, 20)
    dig_P9 = getShort(cal1, 22)

    # 溼度校正（BME280）
    dig_H1 = getUChar(cal2, 0)
    dig_H2 = getShort(cal3, 0)
    dig_H3 = getUChar(cal3, 2)

    dig_H4 = getChar(cal3, 3)
    dig_H4 = (dig_H4 << 24) >> 20
    dig_H4 = dig_H4 | (getChar(cal3, 4) & 0x0F)

    dig_H5 = getChar(cal3, 5)
    dig_H5 = (dig_H5 << 24) >> 20
    dig_H5 = dig_H5 | (getUChar(cal3, 4) >> 4 & 0x0F)

    dig_H6 = getChar(cal3, 6)

    # 等待量測完成（Datasheet 計算）
    wait_time_ms = 1.25 + (2.3 * OVERSAMPLE_TEMP) + ((2.3 * OVERSAMPLE_PRES) + 0.575) + ((2.3 * OVERSAMPLE_HUM) + 0.575)
    time.sleep(wait_time_ms / 1000.0)

    # 讀取原始資料
    data = bus.read_i2c_block_data(addr, REG_DATA, 8)
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw  = (data[6] << 8) | data[7]

    # 溫度補償
    var1 = ((((temp_raw >> 3) - (dig_T1 << 1))) * (dig_T2)) >> 11
    var2 = (((((temp_raw >> 4) - (dig_T1)) * ((temp_raw >> 4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
    t_fine = var1 + var2
    temperature = float(((t_fine * 5) + 128) >> 8) / 100.0  # °C

    # 壓力補償
    var1 = t_fine / 2.0 - 64000.0
    var2 = var1 * var1 * dig_P6 / 32768.0
    var2 = var2 + var1 * dig_P5 * 2.0
    var2 = var2 / 4.0 + dig_P4 * 65536.0
    var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * dig_P1
    if var1 == 0:
        pressure = 0.0
    else:
        pressure = 1048576.0 - pres_raw
        pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
        var1 = dig_P9 * pressure * pressure / 2147483648.0
        var2 = pressure * dig_P8 / 32768.0
        pressure = pressure + (var1 + var2 + dig_P7) / 16.0
        pressure = pressure / 100.0  # hPa

    # 溼度（若 BMP280 會趨近 0）
    humidity = t_fine - 76800.0
    humidity = (hum_raw - (dig_H4 * 64.0 + dig_H5 / 16384.0 * humidity)) * \
               (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * humidity * (1.0 + dig_H3 / 67108864.0 * humidity)))
    humidity = humidity * (1.0 - dig_H1 * humidity / 524288.0)
    if humidity > 100.0:
        humidity = 100.0
    elif humidity < 0.0:
        humidity = 0.0

    return temperature, pressure, humidity

# ===== 氣壓 -> 高度（標準大氣近似）=====
def pressure_to_altitude_m(pressure_hpa: float, sea_level_hpa: float = 1013.25) -> float:
    if pressure_hpa <= 0.0:
        return float('nan')
    return 44330.0 * (1.0 - math.pow(pressure_hpa / sea_level_hpa, 0.190258))

def identity_quaternion() -> Quaternion:
    q = Quaternion()
    q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
    return q

class BaroOdomPublisher(Node):
    def __init__(self):
        super().__init__('baro_odom_publisher')

        # === 參數 ===
        self.declare_parameter('sea_level_pressure_hpa', 1013.25)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('zero_on_start', True)
        self.declare_parameter('smooth_window', 5)   # 平滑視窗大小（預設 5）

        self.P0 = float(self.get_parameter('sea_level_pressure_hpa').get_parameter_value().double_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.zero_on_start = self.get_parameter('zero_on_start').get_parameter_value().bool_value
        self.win = int(self.get_parameter('smooth_window').get_parameter_value().integer_value)
        self.win = max(1, self.win)

        # 發佈到 baro
        self.odom_pub = self.create_publisher(Odometry, 'baro', 10)

        # 平滑緩衝區
        self.alt_buf = deque(maxlen=self.win)

        # 零點
        self.z0 = 0.0
        self.initialized_zero = False

        # 20 Hz
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info(
            f'Baro->Odometry started: frame_id={self.frame_id}, child_frame_id={self.child_frame_id}, '
            f'P0={self.P0:.2f} hPa, zero_on_start={self.zero_on_start}, smooth_window={self.win}'
        )

    def timer_cb(self):
        try:
            _t_c, pressure_hpa, _hum = readBME280All(DEVICE)
        except Exception as e:
            self.get_logger().error(f'Read sensor failed: {e}')
            return

        alt_raw = pressure_to_altitude_m(pressure_hpa, self.P0)
        if math.isnan(alt_raw):
            self.get_logger().warn('Altitude is NaN (pressure invalid).')
            return

        # === 移動視窗平滑 ===
        self.alt_buf.append(alt_raw)
        alt_smooth = sum(self.alt_buf) / len(self.alt_buf)

        # === 啟動零點：等累積到「完整視窗」再設定（z0 = 前 N 筆平均）===
        if self.zero_on_start and not self.initialized_zero and len(self.alt_buf) == self.win:
            self.z0 = alt_smooth
            self.initialized_zero = True
            # 下一輪開始才發佈，避免這一筆同時被當作 z0 又輸出非零
            return

        # z0 尚未設定前不發佈，避免初始抖動
        if not self.initialized_zero:
            return

        z_rel = alt_smooth - self.z0

        # === 組 Odometry ===
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = float(z_rel)
        odom.pose.pose.orientation = identity_quaternion()

        # 速度未知先設 0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # 協方差：x/y 與姿態未知 -> 大；z 給較小
        pose_cov = [0.0]*36
        pose_cov[0]  = 1e6
        pose_cov[7]  = 1e6
        pose_cov[14] = 0.25   # z 約 0.5 m 標準差
        pose_cov[21] = 1e6
        pose_cov[28] = 1e6
        pose_cov[35] = 1e6
        odom.pose.covariance = pose_cov

        twist_cov = [0.0]*36
        for i in (0, 7, 14, 21, 28, 35):
            twist_cov[i] = 1e6
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = BaroOdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
