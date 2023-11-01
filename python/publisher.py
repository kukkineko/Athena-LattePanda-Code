import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from lidar import Lidar  # Import the Lidar class from your 'lidar.py' module

class LaserScanPublisher(Node):
    def __init__(self, topic='scan'):
        super().__init__('laser_scan_publisher')
        self.publisher = self.create_publisher(LaserScan, topic, 15)
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
        laser_scan_msg = self.gen_laser_scan_msg()
        if laser_scan_msg is not None:
            self.get_logger().info(f'Publishing LaserScan at {laser_scan_msg.header.stamp.sec}')
            self.publisher.publish(laser_scan_msg)

    def gen_laser_scan_msg(self):
        scan = self.lidar.scan()
        if scan:
            laser_scan_msg = LaserScan()
            laser_scan_msg.header.stamp = self.get_clock().now().to_msg()
            laser_scan_msg.header.frame_id = 'scan'
            laser_scan_msg.angle_min = self.lidar.angle_min
            laser_scan_msg.angle_max = self.lidar.angle_max
            laser_scan_msg.angle_increment = self.lidar.angle_increment
            laser_scan_msg.time_increment = self.lidar.time_increment
            laser_scan_msg.scan_time = self.lidar.scan_time
            laser_scan_msg.range_min = self.lidar.range_min
            laser_scan_msg.range_max = self.lidar.range_max

            # Use the updated ranges and intensities from Lidar instance
            laser_scan_msg.ranges = self.lidar.ranges_list
            laser_scan_msg.intensities = self.lidar.intensities_list

            return laser_scan_msg
        else:
            return None

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
