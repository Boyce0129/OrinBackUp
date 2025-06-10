#新版腳本用按鍵Q來結束程式，以確保執行緒結束，舊版CTRL+C來結束程式可能會有執行緒未結束的問題(但似乎還是沒解決開太久會自動shutdown的問題)
import threading
import sys
import termios
import tty
import rclpy
from sensor_msgs.msg import NavSatFix
import pynmea2
import time
import serial
from rclpy.node import Node
# 串列通訊設定
port = "/dev/ttyTHS0"
running = True  # 全域變數控制程式運行

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 100)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.serial_port = serial.Serial(port, 115200, timeout=1)

    def timer_callback(self):
        global running
        if not running:
            return  # 若 running=False，則停止執行

        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line.startswith('$GNGGA'):
                parsed_data = pynmea2.parse(line)
                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'gps_frame'
                msg.latitude = parsed_data.latitude
                msg.longitude = parsed_data.longitude
                msg.altitude = parsed_data.altitude
                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error reading GPS data: {e}')

# 🔹 **非阻塞式鍵盤輸入偵測函數**
def get_key():
    """ 讀取鍵盤按鍵，非阻塞式 """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

# 🔹 **鍵盤監聽執行緒**
def keyboard_listener():
    global running
    print("按 Q 來停止程式，不要按CTRL+C (topic如果echo不出來就就重開幾次)")
    while running:
        key = get_key()
        if key.lower() == 'q':  # 偵測 Q 鍵 (不分大小寫)
            print("\n偵測到 Q，正在停止程式...")
            running = False
            rclpy.shutdown()  # 停止 ROS2
            break
        time.sleep(0.1)

def main(args=None):
    global running
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()

    # 🔹 **啟動鍵盤監聽執行緒**
    keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
    keyboard_thread.start()

    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        print("\n⏹ 手動終止程式...")
    finally:
        running = False  # 停止程式
        gps_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
