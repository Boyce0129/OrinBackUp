#!/usr/bin/env python3
import rclpy, time, threading, socket, base64, select
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial
from pyubx2 import UBXReader
import pyproj, numpy as np
from collections import deque
from datetime import datetime, timezone

def _deg_to_nmea(deg: float, is_lat: bool):
    d = int(abs(deg))
    m = (abs(deg) - d) * 60.0
    s = f"{d:02d}{m:07.4f}" if is_lat else f"{d:03d}{m:07.4f}"
    hemi = ("N" if deg >= 0 else "S") if is_lat else ("E" if deg >= 0 else "W")
    return s, hemi

def build_gga(lat: float, lon: float, alt: float) -> str:
    hhmmss = datetime.now(timezone.utc).strftime("%H%M%S")
    lat_s, lat_c = _deg_to_nmea(lat, True)
    lon_s, lon_c = _deg_to_nmea(lon, False)
    body = (
        f"GPGGA,{hhmmss},{lat_s},{lat_c},{lon_s},{lon_c},1,12,1.0,"
        f"{alt:.1f},M,0.0,M,,"
    )
    cks = 0
    for ch in body:
        cks ^= ord(ch)
    return f"${body}*{cks:02X}\r\n"

class RtkGpsNode(Node):
    def __init__(self):
        super().__init__('rtk_gps_node')

        self.declare_parameter('device', '/dev/ttyTHS0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('ntrip_host', '210.241.63.193')
        self.declare_parameter('ntrip_port', 81)
        self.declare_parameter('ntrip_mountpoint', '2020_GNSS')
        self.declare_parameter('ntrip_username', 'uavlab6061')
        self.declare_parameter('ntrip_password', 'iamsupersmart')

        self.device   = self.get_parameter('device').value
        self.baudrate = self.get_parameter('baudrate').value

        try:
            self.serial_port = serial.Serial(self.device, self.baudrate, timeout=1)
            self.ubx_reader  = UBXReader(self.serial_port, protfilter=2)
            self.get_logger().info(f'已連接 {self.device} @ {self.baudrate}')
        except Exception as e:
            self.get_logger().error(f"串口連接失敗: {e}")
            raise

        self.publisher_ = self.create_publisher(Odometry, 'rtk/odom', 10)
        self.serial_lock = threading.Lock()
        self.running = True

        self.current_lat_deg = 0.0
        self.current_lon_deg = 0.0
        self.current_alt_m   = 0.0
        self.num_sats        = 0
        self.hAcc            = 0.0
        self.vAcc            = 0.0

        self.ref_lat = self.ref_lon = self.ref_alt = None
        self.ref_buf = deque(maxlen=10)
        self.ref_ready = False
        self.start_time = time.time()

        self.prev_carr_soln = None

        threading.Thread(target=self.ntrip_thread, daemon=True).start()
        threading.Thread(target=self.read_ubx_thread, daemon=True).start()

    def ntrip_thread(self):
        h = self.get_parameter('ntrip_host').value
        p = self.get_parameter('ntrip_port').value
        m = self.get_parameter('ntrip_mountpoint').value
        u = self.get_parameter('ntrip_username').value
        pw= self.get_parameter('ntrip_password').value

        try:
            sock = socket.socket(); sock.settimeout(10); sock.connect((h, p))
            hdr = (f"GET /{m} HTTP/1.0\r\nUser-Agent: rtk_node\r\n"
                   f"Authorization: Basic {base64.b64encode(f'{u}:{pw}'.encode()).decode()}\r\n"
                   "Ntrip-Version: Ntrip/2.0\r\n\r\n")
            sock.sendall(hdr.encode())

            data = b""
            while not data.endswith(b"\r\n\r\n"):
                data += sock.recv(1)
            if b"200" not in data:
                self.get_logger().error(f"NTRIP 連線失敗:\n{data.decode()}")
                return
            self.get_logger().info("NTRIP 連線成功 (200 OK)")
            sock.setblocking(False)
        except Exception as e:
            self.get_logger().error(f"NTRIP 連線錯誤: {e}")
            return

        sock.sendall(build_gga(0,0,0).encode())
        next_gga = time.time() + 5
        buf = b""

        while rclpy.ok() and self.running:
            if time.time() >= next_gga:
                lat = self.current_lat_deg if self.current_lat_deg else 0.0
                lon = self.current_lon_deg if self.current_lon_deg else 0.0
                alt = self.current_alt_m   if self.current_alt_m   else 0.0
                gga = build_gga(lat, lon, alt).encode()
                sock.sendall(gga)
                next_gga += 5

            r, _, _ = select.select([sock], [], [], 1)
            if not r:
                continue
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    self.get_logger().error("NTRIP 連線被斷開")
                    break
                buf += chunk
                with self.serial_lock:
                    self.serial_port.write(chunk)
            except Exception as e:
                self.get_logger().error(f"NTRIP recv 錯誤: {e}")
                time.sleep(1)

    def read_ubx_thread(self):
        last_acc_print_time = time.time()
        while rclpy.ok() and self.running:
            try:
                with self.serial_lock:
                    raw, parsed = self.ubx_reader.read()
            except Exception as e:
                self.get_logger().error(f"UBX 讀取錯誤: {e}")
                time.sleep(0.1)
                continue

            if not parsed:
                continue

            if parsed.identity == 'NAV-PVT':
                lat_deg = parsed.lat
                lon_deg = parsed.lon
                h_m     = parsed.hMSL / 1e3

                self.current_lat_deg = lat_deg
                self.current_lon_deg = lon_deg
                self.current_alt_m   = h_m
                self.num_sats        = parsed.numSV
                self.hAcc            = getattr(parsed, 'hAcc', 0.0)
                self.vAcc            = getattr(parsed, 'vAcc', 0.0)

                vel_n = parsed.velN / 1e3
                vel_e = parsed.velE / 1e3
                vel_d = parsed.velD / 1e3


                fix_type  = parsed.fixType
                carr_soln = parsed.carrSoln

                if self.prev_carr_soln is not None and carr_soln < self.prev_carr_soln:
                    self.get_logger().warn(f"⚠️ 載波相位解下降：{self.prev_carr_soln} ➜ {carr_soln}")
                elif self.prev_carr_soln is not None and carr_soln > self.prev_carr_soln:
                    self.get_logger().info(f"⬆️ 載波相位解上升：{self.prev_carr_soln} ➜ {carr_soln}")
                self.prev_carr_soln = carr_soln

                # 顯示尚未初始化時的 hAcc / vAcc
                if not self.ref_ready:
                    now = time.time()
                    if now - last_acc_print_time >= 0.5:
                        self.get_logger().info(
                            f"carr_soln={carr_soln}, hAcc={parsed.hAcc/10:.2f} cm, vAcc={parsed.vAcc/10:.2f} cm"
                        )
                        last_acc_print_time = now

                if not self.ref_ready and time.time() - self.start_time > 5:
                    if carr_soln == 2: # 訊號不好時，可用「if carr_soln in (1, 2):」
                        self.ref_buf.append((lat_deg, lon_deg, h_m))
                        if len(self.ref_buf) == self.ref_buf.maxlen:
                            self.ref_lat = np.mean([x[0] for x in self.ref_buf])
                            self.ref_lon = np.mean([x[1] for x in self.ref_buf])
                            self.ref_alt = np.mean([x[2] for x in self.ref_buf])
                            self.ref_ready = True
                            self.get_logger().info(
                                f"✅ 參考點初始化成功 @ FIX: lat={self.ref_lat:.7f}, lon={self.ref_lon:.7f}, alt={self.ref_alt:.2f} "
                                f"| numSV={self.num_sats}, hAcc={self.hAcc/10:.1f}, vAcc={self.vAcc/10:.1f}"
                            )
                    else:
                        self.ref_buf.clear()
                elif self.ref_ready:
                    enu = self.wgs84_to_enu(lat_deg, lon_deg, h_m,
                                            self.ref_lat, self.ref_lon, self.ref_alt)

                    msg = Odometry()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'enu_frame'
                    msg.pose.pose.position.x = float(enu[0])
                    msg.pose.pose.position.y = float(enu[1])
                    msg.pose.pose.position.z = float(enu[2])
                    msg.pose.pose.orientation.w = 1.0
                    msg.twist.twist.linear.x = float(vel_e)
                    msg.twist.twist.linear.y = float(vel_n)
                    msg.twist.twist.linear.z = float(-vel_d)
                    self.publisher_.publish(msg)

                    self.get_logger().info(
                        f"ENU=({enu[0]:>6.2f}, {enu[1]:>6.2f}, {enu[2]:>6.2f})  "
                        f"vel=({vel_e:>6.2f}, {vel_n:>6.2f}, {-vel_d:>6.2f})  "
                        f"fix={fix_type:<1} carr={carr_soln:<1}  "
                        f"numSV={self.num_sats:>2}, hAcc={self.hAcc/10:>5.3f}, vAcc={self.vAcc/10:>5.3f}"
                    )

    def wgs84_to_enu(self, lat, lon, h, ref_lat, ref_lon, ref_h):
        wgs84 = pyproj.CRS('epsg:4326')
        ecef  = pyproj.CRS('epsg:4978')
        transformer = pyproj.Transformer.from_crs(wgs84, ecef, always_xy=True)
        x,  y,  z  = transformer.transform(lon, lat, h)
        xr, yr, zr = transformer.transform(ref_lon, ref_lat, ref_h)
        dx, dy, dz = x - xr, y - yr, z - zr
        phi = np.deg2rad(ref_lat); lam = np.deg2rad(ref_lon)
        ned = np.array([
            -np.sin(phi)*np.cos(lam)*dx - np.sin(phi)*np.sin(lam)*dy + np.cos(phi)*dz,
            -np.sin(lam)*dx + np.cos(lam)*dy,
            -np.cos(phi)*np.cos(lam)*dx - np.cos(phi)*np.sin(lam)*dy - np.sin(phi)*dz
        ])
        return np.array([ned[1], ned[0], -ned[2]])

    def destroy_node(self):
        self.running = False
        time.sleep(0.5)
        if self.serial_port.is_open:
            self.serial_port.close()
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