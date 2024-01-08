import rospy
import roslaunch
import time

def start_ros_master():
    # Initialize ROS node
    rospy.init_node('ros_master_starter', anonymous=True)

    # Specify the roscore launch file
    roscore_launch_file = roslaunch.rlutil.resolve_launch_arguments(['roscore'])[0]

    # Create a roslaunch parent with the roscore launch file
    parent = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'), [(roscore_launch_file, [])], is_core=True)

    try:
        # Start the roscore
        parent.start()

        # Allow some time for the roscore to start
        time.sleep(2)

        # Print a message indicating that the roscore is running
        rospy.loginfo('ROS Master (roscore) is running.')

        # Keep the script running to maintain the roscore
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        # Shutdown the roslaunch parent when the script is interrupted
        parent.shutdown()

if __name__ == '__main__':
    start_ros_master()
