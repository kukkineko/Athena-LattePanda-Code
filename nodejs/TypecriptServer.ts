import * as rclnodejs from 'rclnodejs';
import { LaserScan } from 'sensor_msgs/msg/LaserScan';
import { Imu } from 'sensor_msgs/msg/Imu';

// Create a ROS 2 node
rclnodejs.init().then(() => {
    const node = rclnodejs.createNode('typescript_server_node');

    // Create publishers for lidar and imu topics
    const lidarPublisher = node.createPublisher(LaserScan, 'lidar_topic');
    const imuPublisher = node.createPublisher(Imu, 'imu_topic');

    // Function to handle received data and publish to ROS 2
    function handleData(topic: string, data: any) {
        switch (topic) {
            case 'lidar':
                publishLidarData(data);
                break;
            case 'imu':
                publishImuData(data);
                break;
            default:
                console.error(`Unknown topic: ${topic}`);
        }
    }

    // Function to publish lidar data
    function publishLidarData(data: any) {
        // Assuming data is in the format suitable for LaserScan message
        const lidarMessage = new LaserScan();
        lidarMessage.header.stamp.sec = data.header.stamp.sec;
        lidarMessage.header.stamp.nanosec = data.header.stamp.nanosec;
        lidarMessage.header.frame_id = data.header.frame_id;
        lidarMessage.angle_min = data.angle_min;
        lidarMessage.angle_max = data.angle_max;
        lidarMessage.angle_increment = data.angle_increment;
        lidarMessage.time_increment = data.time_increment;
        lidarMessage.scan_time = data.scan_time;
        lidarMessage.range_min = data.range_min;
        lidarMessage.range_max = data.range_max;
        lidarMessage.ranges = data.ranges;
        lidarMessage.intensities = data.intensities;

        lidarPublisher.publish(lidarMessage);
    }

    // Function to publish imu data
    function publishImuData(data: any) {
        // Assuming data is in the format suitable for Imu message
        const imuMessage = new Imu();
        imuMessage.header.stamp.sec = data.header.stamp.sec;
        imuMessage.header.stamp.nanosec = data.header.stamp.nanosec;
        imuMessage.header.frame_id = data.header.frame_id;
        imuMessage.orientation = data.orientation;
        imuMessage.angular_velocity = data.angular_velocity;
        imuMessage.linear_acceleration = data.linear_acceleration;

        imuPublisher.publish(imuMessage);
    }

    // Handle termination signals
    process.on('SIGINT', handleExit);
    process.on('SIGTERM', handleExit);

    // Subscribe to data from the Python client
    const subscription = node.createSubscription(
        'python_data_topic',
        'std_msgs/msg/String',
        (msg: rclnodejs.Message) => {
            if (msg instanceof rclnodejs.Message) {
                // Handle specific message types (e.g., LaserScan, Imu)
                const data = JSON.parse((msg as any).data);
                handleData(data.topic, data.data);
            } else if (typeof msg === 'string') {
                // Handle string messages
                console.log('Received string message:', msg);
            } else {
                // Handle other types
                console.error('Received unsupported message type:', msg);
            }
        }
    );

    // Spin the node to handle messages
    rclnodejs.spin(node);

    // Cleanup on script termination
    process.on('exit', () => {
        handleExit();
    });

    function handleExit() {
        console.log("Received termination signal. Cleaning up...");
        // Additional cleanup logic if needed
        process.exit(0);
    }
});
