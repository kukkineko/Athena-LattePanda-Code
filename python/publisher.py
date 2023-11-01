import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from lidar import Lidar  # Import the Lidar class from your 'lidar.py' module

class LaserScanPublisher(Node):
    def __init__(self, topic='laser_frame'):
        super().__init__('laser_scan_publisher')
        self.publisher = self.create_publisher(LaserScan, topic, 10)
        self.timer = None
        self.is_running = False
        self.lidar = Lidar()  # Initialize the Lidar instance

    def start(self, interval=1.0):
        if self.is_running:
            return

        self.is_running = True
        self.timer = self.create_timer(interval, self.publish_laser_scan)

    def stop(self):
        if self.timer:
            self.timer.cancel()
            self.is_running = False

    def publish_laser_scan(self):
        msg = self.gen_laser_scan_msg()
        self.get_logger().info(f'Publish msg at {msg.header.stamp.sec}')
        self.publisher.publish(msg)

    def gen_laser_scan_msg(self):
        scan = self.lidar.scan()
        if scan != True:
            #scan unsuccessful, can't generate laser scan message.
            return None
           
        laser_scan_msg = LaserScan()
        laser_scan_msg.header.stamp = self.get_clock().now().to_msg()
        laser_scan_msg.header.frame_id = 'laser_frame'
        laser_scan_msg.angle_min = self.lidar.angle_min
        laser_scan_msg.angle_max = self.lidar.angle_max
        laser_scan_msg.angle_increment = self.lidar.angle_increment
        laser_scan_msg.time_increment = self.lidar.time_increment
        laser_scan_msg.scan_time = self.lidar.scan_time
        laser_scan_msg.range_min = self.lidar.range_min
        laser_scan_msg.range_max = self.lidar.range_max
        laser_scan_msg.ranges = self.lidar.grid
        laser_scan_msg.intensities = self.lidar.intensity


        return laser_scan_msg

def main(args=None):
    rclpy.init(args=args)
    laser_scan_publisher = LaserScanPublisher()

    try:
        laser_scan_publisher.start()
        rclpy.spin(laser_scan_publisher)
    except KeyboardInterrupt:
        laser_scan_publisher.stop()
    except Exception as e:
        laser_scan_publisher.get_logger().error(str(e))

    laser_scan_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
