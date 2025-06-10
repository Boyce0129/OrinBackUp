from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import pynmea2
import rclpy
import time
import serial

port = "/dev/ttyTHS0"
#若使用USB port口，需把THS改成USB
class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps', 100)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.serial_port = serial.Serial(port, 57600, timeout=1)

    def timer_callback(self):
        start_time = time.time()
        try:
            # 记录读取数据前的时间
            line = self.serial_port.readline().decode('utf-8').strip()
            if line.startswith('$GNGGA'): #GNGGA為GPS輸出數據的一種格式，當模組包有把GPS的輸出格式設成GNGGA時，這邊ROS2配置檔才能接收
                  # 记录读取数据后的时
                parsed_data = pynmea2.parse(line)
                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg() #設定時間戳記，用於同步多個感測器數據、確保使用最新數據、日後回溯與分析
                msg.header.frame_id = 'gps_frame'
                msg.latitude = parsed_data.latitude
                msg.longitude = parsed_data.longitude
                msg.altitude = parsed_data.altitude
               
                self.publisher_.publish(msg)
                publish_time = time.time()  # 记录发布数据后的时间
                #self.get_logger().info(f'Publishing GPS Data: {line}')
                read_time = time.time()
                #self.get_logger().info(f'Read time: {read_time - start_time} seconds')
                #self.get_logger().info(f'Publish time: {publish_time - read_time} seconds')
                #print(msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f'Error reading GPS data: {e}')
        read_time = time.time()
        #self.get_logger().info(f'Read time: {read_time - start_time} seconds')

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

