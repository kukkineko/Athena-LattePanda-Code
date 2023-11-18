import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
import multiprocessing
from lidar import Lidar
from IMU import IMU

class LaserScanPublisher(Node):
    def __init__(self, topic='scan'):
        super().__init__('laser_scan_publisher')
        self.publisher = self.create_publisher(LaserScan, topic, 60)
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

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(Imu, 'imu', 60)
        self.timer = None
        self.is_running = False
        self.imu = IMU()

    def start(self, interval=1.0):
        if self.is_running:
            return

        self.is_running = True
        self.timer = self.create_timer(interval, self.publish_imu)

    def stop(self):
        if self.timer:
            self.timer.cancel()
            self.is_running = False

    def publish_imu(self):
        self.imu.read()
        ax, ay, az = float(self.imu.ax), float(self.imu.ay), float(self.imu.az)
        gx, gy, gz = float(self.imu.gx), float(self.imu.gy), float(self.imu.gz)

        #generate imu message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu'
        imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        imu_msg.angular_velocity = Vector3(x=gx, y=gy, z=gz)
        imu_msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)
    
        self.get_logger().info(f'Publishing IMU at {imu_msg.header.stamp.sec}')
        self.publisher.publish(imu_msg)




def lidar_publisher_process():
    rclpy.init()
    laser_scan_publisher = LaserScanPublisher()

    try:
        laser_scan_publisher.start()
        rclpy.spin(laser_scan_publisher)

    except Exception as e:
        laser_scan_publisher.get_logger().error(str(e))

    finally:
        laser_scan_publisher.destroy_node()
        rclpy.shutdown()

def imu_publisher_process():
    rclpy.init()
    imu_publisher = ImuPublisher()

    try:
        imu_publisher.start()
        rclpy.spin(imu_publisher)

    except Exception as e:
        imu_publisher.get_logger().error(str(e))

    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()


def main():
    # Create processes for Lidar and Odometry publishers
    lidar_process = multiprocessing.Process(target=lidar_publisher_process)
    imu_process = multiprocessing.Process(target=imu_publisher_process)

    try:
        lidar_process.start()
        imu_process.start()

        lidar_process.join()
        imu_process.join()


    except KeyboardInterrupt:
        lidar_process.terminate()
        imu_process.terminate()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
