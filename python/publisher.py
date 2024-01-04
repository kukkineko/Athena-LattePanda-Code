import rospy
from sensor_msgs.msg import LaserScan
from lidar import Lidar  # Import the Lidar class from your 'lidar.py' module

class LaserScanPublisher:
    def __init__(self, topic='scan'):
        rospy.init_node('laser_scan_publisher')
        self.publisher = rospy.Publisher(topic, LaserScan, queue_size=10)
        self.rate = rospy.Rate(120)
        self.is_running = False
        self.lidar = Lidar()  # Initialize the Lidar instance

    def start(self):
        if self.is_running:
            return

        self.is_running = True
        self.publish_laser_scan()

    def stop(self):
        self.is_running = False

    def publish_laser_scan(self):
        while not rospy.is_shutdown() and self.is_running:
            laser_scan_msg = self.gen_laser_scan_msg()
            if laser_scan_msg is not None:
                rospy.loginfo(f'Publishing LaserScan at {rospy.Time.now()}')
                self.publisher.publish(laser_scan_msg)
            self.rate.sleep()

    def gen_laser_scan_msg(self):
        scan = self.lidar.scan()
        if scan:
            laser_scan_msg = LaserScan()
            laser_scan_msg.header.stamp = rospy.Time.now()
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

def main():
    laser_scan_publisher = LaserScanPublisher()

    try:
        laser_scan_publisher.start()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
