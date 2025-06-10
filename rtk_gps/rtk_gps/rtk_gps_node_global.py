import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # 修改為 NavSatFix 訊息類型
from sensor_msgs.msg import NavSatStatus
import serial
from pyubx2 import UBXReader
import threading
from ntrip_client import NTRIPClient
import pynmea2

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
        
        # 創建 NavSatFix 訊息發布者
        self.publisher_ = self.create_publisher(NavSatFix, 'rtk/fix', 10)  # 修改為 NavSatFix，topic 改為 rtk/fix
        
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
                        
                        # 獲取定位類型和載波相位解
                        fix_type = parsed_data.fixType
                        carr_soln = parsed_data.carrSoln
                        
                        # 創建 NavSatFix 訊息
                        msg = NavSatFix()
                        
                        # 填充時間戳與坐標系
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'gps_frame'  # 表示全球座標系
                        
                        # 填充經緯高
                        msg.latitude = lat
                        msg.longitude = lon
                        msg.altitude = height
                        
                        # 填充定位狀態
                        if fix_type == 0:
                            msg.status.status = NavSatStatus.STATUS_NO_FIX
                        elif fix_type == 1:
                            msg.status.status = NavSatStatus.STATUS_FIX
                        elif fix_type == 2:
                            msg.status.status = NavSatStatus.STATUS_SBAS_FIX
                        elif fix_type in [3, 4]:
                            if carr_soln == 0:
                                msg.status.status = NavSatStatus.STATUS_GBAS_FIX  # 差分定位但無 RTK
                            elif carr_soln == 1:
                                msg.status.status = NavSatStatus.STATUS_GBAS_FIX  # RTK 浮點解
                            elif carr_soln == 2:
                                msg.status.status = NavSatStatus.STATUS_GBAS_FIX  # RTK 固定解
                        msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
                        
                        # 發布訊息
                        self.publisher_.publish(msg)
                        self.get_logger().info(
                            f"發布訊息：經緯高=({lat:.7f}, {lon:.7f}, {height:.3f}), "
                            f"定位類型={fix_type}, 載波相位解={carr_soln}, 衛星數={self.num_sats}"
                        )
                    elif parsed_data.identity == 'RXM-RTCM':
                        self.get_logger().debug(f"接收到 RTCM 訊息：{parsed_data}")
            except Exception as e:
                self.get_logger().error(f"讀取 UBX 數據時發生錯誤：{str(e)}")
                time.sleep(0.1)  # 避免過於頻繁的錯誤日誌

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
