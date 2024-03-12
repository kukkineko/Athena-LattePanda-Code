import rospy
from sensor_msgs.msg import LaserScan, Imu
from lidar import Lidar  # Import the Lidar class from your 'lidar.py' module
from IMU import IMU  # Import the IMU class from your 'IMU.py' module
import tf

class LaserScanPublisher:
    def __init__(self, topic_laser='scan', topic_imu='imu'):
        rospy.init_node('imu_laser_publisher', anonymous=True)
        self.publisher_laser = rospy.Publisher(topic_laser, LaserScan, queue_size=10)
        self.publisher_imu = rospy.Publisher(topic_imu, Imu, queue_size=10)
        self.rate = rospy.Rate(120)
        self.is_running = False
        #self.lidar = Lidar()  # Initialize the Lidar instance
        self.imu = IMU()  # Initialize the IMU instance

        # Initialize TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

    def start(self):
        if self.is_running:
            return

        self.is_running = True
        self.publish_sensor_data()

    def stop(self):
        self.is_running = False

    def publish_sensor_data(self):
        try:
            while not rospy.is_shutdown() and self.is_running:
                # Publish LaserScan data
                # laser_scan_msg = self.gen_laser_scan_msg()
                # if laser_scan_msg is not None:
                #     rospy.loginfo('Publishing LaserScan at {}'.format(rospy.Time.now()))
                #     self.publisher_laser.publish(laser_scan_msg)

                # Publish IMU data
                imu_msg = self.gen_imu_msg()
                if imu_msg is not None:
                    rospy.loginfo('Publishing IMU data at {}'.format(rospy.Time.now()))
                    self.publisher_imu.publish(imu_msg)


                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass

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
            laser_scan_msg.ranges = self.lidar.ranges_list
            laser_scan_msg.intensities = self.lidar.intensities_list

            return laser_scan_msg
        else:
            return None



    def gen_imu_msg(self):
        try:
            self.imu.read()
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = 'imu'
            
            # Quaternion orientation data
            imu_msg.orientation.x = float(q[0])  # Replace q[0] with the actual value from your IMU
            imu_msg.orientation.y = float(q[1])  # Replace q[1] with the actual value from your IMU
            imu_msg.orientation.z = float(q[2])  # Replace q[2] with the actual value from your IMU
            imu_msg.orientation.w = float(q[3])  # Replace q[3] with the actual value from your IMU
            
            # Angular velocity (gyro) values
            imu_msg.angular_velocity.x = float(self.imu.gx)
            imu_msg.angular_velocity.y = float(self.imu.gy)
            imu_msg.angular_velocity.z = float(self.imu.gz)
            
            # Linear acceleration (accelerometer) values
            imu_msg.linear_acceleration.x = float(self.imu.ax)
            imu_msg.linear_acceleration.y = float(self.imu.ay)
            imu_msg.linear_acceleration.z = float(self.imu.az)

            return imu_msg
        except:
            pass


def main():
    laser_scan_publisher = LaserScanPublisher()

    try:
        laser_scan_publisher.start()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
