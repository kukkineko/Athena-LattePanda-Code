import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, Imu

class Transformer:
    def __init__(self):
        rospy.init_node('transformer_node')

        # Initialize TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Set your frame IDs
        self.chassis_frame_id = 'chassis'
        self.scan_frame_id = 'base_scan'
        self.imu_frame_id = 'imu_frame'  # Update with your actual IMU frame

        # Subscribe to chassis, scan, and IMU topics
        rospy.Subscriber('chassis', PoseStamped, self.chassis_callback)
        rospy.Subscriber('scan', LaserScan, self.scan_callback)
        rospy.Subscriber('imu', Imu, self.imu_callback)

        rospy.spin()

    def chassis_callback(self, pose_msg):
        # Function to transform chassis pose and broadcast
        transform = self.create_transform(pose_msg.pose, self.chassis_frame_id)
        self.broadcast_transform(transform)

    def scan_callback(self, scan_msg):
        # Function to transform laser scan and broadcast
        transform = self.create_transform(scan_msg.header.frame_id, self.scan_frame_id)
        self.broadcast_transform(transform)

    def imu_callback(self, imu_msg):
        # Function to transform IMU data and broadcast
        transform = self.create_transform(imu_msg.header.frame_id, self.imu_frame_id)
        self.broadcast_transform(transform)

    def create_transform(self, source_frame, target_frame):
        # Create a transform from source frame to target frame
        transform = tf2_ros.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = source_frame
        transform.child_frame_id = target_frame
        transform.transform.translation.x = 0.0  # Adjust as needed
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0  # Adjust as needed
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        return transform

    def broadcast_transform(self, transform):
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    transformer = Transformer()
