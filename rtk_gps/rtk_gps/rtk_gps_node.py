import rclpy
import time
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial
from pyubx2 import UBXReader
import threading
from ntrip_client import NTRIPClient
import pynmea2
import pyproj
import numpy as np
from collections import deque

class RtkGpsNode(Node):
    def __init__(self):
        super().__init__('rtk_gps_node')
        
        # 宣告參數：串口設備與波特率
        self.declare_parameter('device', '/dev/ttyTHS0')
        self.declare_parameter('baudrate', 115200)
        
        # 宣告 NTRIP 參數
        self.declare_parameter('ntrip_host', '210.241.63.193')
        self.declare_parameter('ntrip_port', 81)
        self.declare_parameter('ntrip_mountpoint', '2020_GNSS')
        self.declare_parameter('ntrip_username', 'uavlab6061')
        self.declare_parameter('ntrip_password', 'iamsupersmart')
        self.declare_parameter('ntrip_version', 'Ntrip/1.0')
        
        # 獲取參數值
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # 創建 Odometry 訊息發布者
        self.publisher_ = self.create_publisher(Odometry, 'rtk/odom', 10)
        
        # 初始化串口
        try:
            self.serial_port = serial.Serial(self.device, self.baudrate, timeout=1)
            self.ubx_reader = UBXReader(self.serial_port, protfilter=2)  # 只解析 UBX 協議
            self.get_logger().debug(f"已連接到 {self.device}，波特率 {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"無法連接到 {self.device}：{str(e)}")
            return
        
        # 運行標誌
        self.running = True
        
        # 用於存儲當前經緯度
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.num_sats = 0
        
        # ENU 參考點相關變數
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
        self.ref_buffer = deque(maxlen=10)  # 儲存 10 組數據用於平均
        self.ref_buffer_ready = False  # 標記參考點是否準備好
        
        # 啟動 NTRIP 線程
        self.ntrip_thread = threading.Thread(target=self.ntrip_client_thread)
        self.ntrip_thread.daemon = True
        self.ntrip_thread.start()
        
        # 啟動串口讀取線程
        self.read_thread = threading.Thread(target=self.read_ubx_thread)
        self.read_thread.daemon = True
        self.read_thread.start()

    def ntrip_client_thread(self):
        """NTRIP 線程：從伺服器接收 RTCM 數據並發送到串口"""
        # 獲取 NTRIP 參數
        host = self.get_parameter('ntrip_host').get_parameter_value().string_value
        port = self.get_parameter('ntrip_port').get_parameter_value().integer_value
        mountpoint = self.get_parameter('ntrip_mountpoint').get_parameter_value().string_value
        username = self.get_parameter('ntrip_username').get_parameter_value().string_value
        password = self.get_parameter('ntrip_password').get_parameter_value().string_value
        ntrip_version = self.get_parameter('ntrip_version').get_parameter_value().string_value
        
        # 初始化 NTRIP 客戶端，按順序傳遞參數
        ntrip_client = NTRIPClient(
            host,
            port,
            mountpoint,
            ntrip_version,
            username,
            password,
            logerr=self.get_logger().error,
            logwarn=self.get_logger().warning,
            loginfo=self.get_logger().info,
            logdebug=self.get_logger().debug
        )
        
        # 連接到 NTRIP 伺服器
        if not ntrip_client.connect():
            self.get_logger().error("無法連接到 NTRIP 伺服器")
            return
        
        self.get_logger().debug("成功連接到 NTRIP 伺服器")
        
        # 定期發送 NMEA GGA 句子
        last_nmea_send_time = 0
        
        while rclpy.ok() and self.running:
            try:
                # 每 5 秒發送一次 NMEA GGA 句子
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - last_nmea_send_time >= 5.0:
                    # 如果設備尚未獲得有效定位，使用 u-center 提供的經緯度
                    lat = f"{abs(self.current_lat):.7f}" if self.current_lat != 0 else "2412.268234"
                    lon = f"{abs(self.current_lon):.7f}" if self.current_lon != 0 else "12067.571920"
                    lat_dir = 'N' if self.current_lat >= 0 else 'S'
                    lon_dir = 'E' if self.current_lon >= 0 else 'W'
                    alt = f"{self.current_alt:.1f}" if self.current_alt != 0 else "98.2"
                    
                    # 使用 pynmea2 生成有效的 GGA 句子
                    gga = pynmea2.GGA(
                        'GP', 'GGA',
                        (
                            time.strftime("%H%M%S", time.gmtime()),  # 時間
                            lat,  # 緯度
                            lat_dir,
                            lon,  # 經度
                            lon_dir,
                            '2',  # 定位質量（設為 2，表示差分定位）
                            f"{self.num_sats:02d}" if self.num_sats != 0 else '15',  # 衛星數
                            '1.4',  # HDOP（從 u-center 獲取）
                            alt,  # 高度
                            'M',  # 高度單位
                            '0.0',  # 地球橢球高度
                            'M',  # 地球橢球高度單位
                            '',  # 差分時間
                            ''  # 差分站 ID
                        )
                    )
                    nmea_gga = str(gga) + '\r\n'
                    ntrip_client.send_nmea(nmea_gga)
                    self.get_logger().debug(f"已發送 NMEA GGA 句子到 NTRIP 伺服器: {nmea_gga.strip()}")
                    last_nmea_send_time = current_time
                
                # 從 NTRIP 伺服器獲取 RTCM 數據
                rtcm_data = ntrip_client.recv_rtcm()
                if isinstance(rtcm_data, bytes) and rtcm_data:
                    # 確保 rtcm_data 是 bytes 類型且不為空
                    self.serial_port.write(rtcm_data)
                    self.get_logger().debug(f"已將 {len(rtcm_data)} 字節 RTCM 數據發送到 simplertk2b micro")
                    self.get_logger().debug(f"Successfully received {len(rtcm_data)} bytes of RTCM data")
                else:
                    self.get_logger().debug("未收到 RTCM 數據")
            except Exception as e:
                self.get_logger().error(f"NTRIP 客戶端錯誤：{str(e)}")
                time.sleep(1)
                continue
        
        ntrip_client.disconnect()

    def read_ubx_thread(self):
        """串口讀取線程：讀取並解析 UBX 訊息"""
        while rclpy.ok() and self.running:
            try:
                # 從串口讀取 UBX 數據
                (raw_data, parsed_data) = self.ubx_reader.read()
                if parsed_data:
                    if parsed_data.identity == 'NAV-PVT':
                        # 更新當前經緯度
                        lat = parsed_data.lat
                        lon = parsed_data.lon
                        height = parsed_data.hMSL / 1e3  # 單位從毫米轉為米
                        self.current_lat = lat
                        self.current_lon = lon
                        self.current_alt = height
                        # 更新衛星數
                        self.num_sats = parsed_data.numSV
                        
                        # 獲取速度（NED 座標系）
                        vel_n = parsed_data.velN / 1e3  # m/s
                        vel_e = parsed_data.velE / 1e3  # m/s
                        vel_d = parsed_data.velD / 1e3  # m/s
                        
                        # 獲取定位類型和載波相位解
                        fix_type = parsed_data.fixType
                        carr_soln = parsed_data.carrSoln
                        
                        # 初始化參考點
                        if not self.ref_buffer_ready:
                            if fix_type == 3 and (carr_soln == 1 or carr_soln == 2) and self.num_sats >= 10:
                                self.ref_buffer.append((lat, lon, height))
                                if len(self.ref_buffer) == self.ref_buffer.maxlen:
                                    # 計算平均值作為參考點
                                    self.ref_lat = np.mean([x[0] for x in self.ref_buffer])
                                    self.ref_lon = np.mean([x[1] for x in self.ref_buffer])
                                    self.ref_alt = np.mean([x[2] for x in self.ref_buffer])
                                    self.ref_buffer_ready = True
                                    self.get_logger().debug(
                                        f"參考點已初始化：緯度={self.ref_lat:.7f}, 經度={self.ref_lon:.7f}, 高度={self.ref_alt:.3f}"
                                    )
                            continue
                        
                        # 計算 ENU 座標
                        enu = self.wgs84_to_enu(lat, lon, height, self.ref_lat, self.ref_lon, self.ref_alt)
                        
                        # 創建 Odometry 訊息
                        msg = Odometry()
                        
                        # 填充位置（ENU 座標系）
                        msg.pose.pose.position.x = enu[0]  # East
                        msg.pose.pose.position.y = enu[1]  # North
                        msg.pose.pose.position.z = enu[2]  # Up
                        msg.pose.pose.orientation.w = 1.0  # 單位四元數（無旋轉）
                        
                        # 填充速度（ENU 座標系）
                        msg.twist.twist.linear.x = vel_e   # East
                        msg.twist.twist.linear.y = vel_n   # North
                        msg.twist.twist.linear.z = -vel_d  # Up (NED 的 Down 取反)
                        
                        # 填充時間戳與坐標系
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'enu_frame'
                        
                        # 發布訊息
                        self.publisher_.publish(msg)
                        self.get_logger().info(
                            f"發布訊息：ENU 位置=({enu[0]:.3f}, {enu[1]:.3f}, {enu[2]:.3f}), "
                            f"速度=({vel_e:.3f}, {vel_n:.3f}, {-vel_d:.3f}), "
                            f"定位類型={fix_type}, 載波相位解={carr_soln}, 衛星數={self.num_sats}"
                        )
                    elif parsed_data.identity == 'RXM-RTCM':
                        self.get_logger().debug(f"接收到 RTCM 訊息：{parsed_data}")
            except Exception as e:
                self.get_logger().error(f"讀取 UBX 數據時發生錯誤：{str(e)}")
                time.sleep(0.1)  # 避免過於頻繁的錯誤日誌

    def wgs84_to_enu(self, lat, lon, height, ref_lat, ref_lon, ref_height):
        """將 WGS84 經緯高轉換為 ENU 座標"""
        # 定義投影
        wgs84 = pyproj.CRS('epsg:4326')  # WGS84 經緯度
        ecef = pyproj.CRS('epsg:4978')   # ECEF 坐標系
        
        # 創建 Transformer 對象
        transformer = pyproj.Transformer.from_crs(wgs84, ecef, always_xy=True)
        
        # 將參考點和當前點從 WGS84 轉換為 ECEF
        ref_x, ref_y, ref_z = transformer.transform(ref_lon, ref_lat, ref_height)
        x, y, z = transformer.transform(lon, lat, height)
        
        # 計算 ECEF 坐標差
        dx = x - ref_x
        dy = y - ref_y
        dz = z - ref_z
        
        # 將差值轉換為 NED 坐標（中間步驟）
        phi = np.deg2rad(ref_lat)
        lam = np.deg2rad(ref_lon)
        ned = np.array([
            -np.sin(phi) * np.cos(lam) * dx - np.sin(phi) * np.sin(lam) * dy + np.cos(phi) * dz,  # North
            -np.sin(lam) * dx + np.cos(lam) * dy,  # East
            -np.cos(phi) * np.cos(lam) * dx - np.cos(phi) * np.sin(lam) * dy - np.sin(phi) * dz  # Down
        ])
        
        # 將 NED 轉換為 ENU
        enu = np.array([
            ned[1],  # East
            ned[0],  # North
            -ned[2]  # Up (NED 的 Down 取反)
        ])
        return enu

    def destroy_node(self):
        """節點銷毀時關閉串口"""
        self.running = False  # 停止線程
        if self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().debug("串口已關閉")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RtkGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
