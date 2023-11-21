import * as rclnodejs from 'rclnodejs';

// Define ROS 2 message types
const LaserScan = 'sensor_msgs/msg/LaserScan';
const Imu = 'sensor_msgs/msg/Imu';

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
        const lidarMessage = new rclnodejs.Message(LaserScan, {
            header: {
                stamp: { sec: data.header.stamp.sec, nanosec: data.header.stamp.nanosec },
                frame_id: data.header.frame_id,
            },
            angle_min: data.angle_min,
            angle_max: data.angle_max,
            angle_increment: data.angle_increment,
            time_increment: data.time_increment,
            scan_time: data.scan_time,
            range_min: data.range_min,
            range_max: data.range_max,
            ranges: data.ranges,
            intensities: data.intensities,
        });

        lidarPublisher.publish(lidarMessage);
    }


    // Function to publish imu data
    function publishImuData(data: any) {
        // Assuming data is in the format suitable for Imu message
        const imuMessage = new rclnodejs.Message(Imu, {
            header: { stamp: { sec: data.timestamp, nanosec: 0 }, frame_id: 'imu_frame' },
            orientation: data.orientation,
            angular_velocity: data.angular_velocity,
            linear_acceleration: data.linear_acceleration,
        });

        imuPublisher.publish(imuMessage);
    }

    // Handle termination signals
    process.on('SIGINT', handleExit);
    process.on('SIGTERM', handleExit);

    // Subscribe to data from the Python client
    const subscription = node.createSubscription('python_data_topic', 'std_msgs/msg/String', (msg) => {
        try {
            const data = JSON.parse(msg.data);
            handleData(data.topic, data.data);
        } catch (error) {
            console.error(`Error parsing data: ${error}`);
        }
    });

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
