#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan


class Transformer:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('transformer_node')

        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Define topic names and frame IDs
        self.source_topic = "scan"
        self.output_topic = "base_scan"
        self.source_frame = "primesense1_depth_frame"
        self.target_frame = "base_link"

        # Create ROS subscribers and publishers
        self.scan_subscriber = rospy.Subscriber(self.source_topic, LaserScan, self.scan_callback)
        self.base_scan_publisher = rospy.Publisher(self.output_topic, LaserScan, queue_size=10)

    def scan_callback(self, scan_msg):
        try:
            # Transform the scan message to the target frame
            transformed_scan = self.transform_scan(scan_msg)
            
            # Publish the transformed scan
            self.base_scan_publisher.publish(transformed_scan)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF Transformation Error: {}".format(e))

    def transform_scan(self, scan_msg):
        # Create a new LaserScan message
        transformed_scan = LaserScan()
        transformed_scan.header = scan_msg.header

        try:
            # Transform scan frame to the target frame
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rospy.Time(0))
            transformed_scan.header.frame_id = self.target_frame

            # Transform the scan points
            for point in scan_msg.ranges:
                transformed_point = self.transform_point(point, transform)
                transformed_scan.ranges.append(transformed_point)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF Transformation Error: {}".format(e))

        return transformed_scan


    def transform_point(self, point, transform):
        # Transform a single point from the source to target frame

        # Create a geometry_msgs/PointStamped message for the point
        point_stamped = geometry_msgs.msg.PointStamped()
        point_stamped.header.frame_id = self.source_frame
        point_stamped.header.stamp = rospy.Time()  # Use latest transform available

        # Set the point coordinates
        point_stamped.point.x = point
        point_stamped.point.y = 0.0  # Assuming 1D laser scan, adjust if needed
        point_stamped.point.z = 0.0  # Assuming 1D laser scan, adjust if needed

        # Transform the point using the provided transform
        transformed_point = self.tf_buffer.transform(point_stamped, self.target_frame)

        return transformed_point.point.x  # Assuming 1D laser scan, adjust if needed

if __name__ == '__main__':
    try:
        transformer = Transformer()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
