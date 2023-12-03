"use strict";
exports.__esModule = true;
var rclnodejs = require("rclnodejs");
var LaserScan_1 = require("rosidl-typescript-msgs/sensor_msgs/msg/LaserScan");
var Imu_1 = require("rosidl-typescript-msgs/sensor_msgs/msg/Imu");
// Create a ROS 2 node
rclnodejs.init().then(function () {
    var node = rclnodejs.createNode('typescript_server_node');
    // Create publishers for lidar and imu topics
    var lidarPublisher = node.createPublisher(LaserScan_1.LaserScan, 'lidar_topic');
    var imuPublisher = node.createPublisher(Imu_1.Imu, 'imu_topic');
    // Function to handle received data and publish to ROS 2
    function handleData(topic, data) {
        switch (topic) {
            case 'lidar':
                publishLidarData(data);
                break;
            case 'imu':
                publishImuData(data);
                break;
            default:
                console.error("Unknown topic: ".concat(topic));
        }
    }
    // Function to publish lidar data
    function publishLidarData(data) {
        // Assuming data is in the format suitable for LaserScan message
        var lidarMessage = new LaserScan_1.LaserScan();
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
    function publishImuData(data) {
        // Assuming data is in the format suitable for Imu message
        var imuMessage = new Imu_1.Imu();
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
    var subscription = node.createSubscription('python_data_topic', 'std_msgs/msg/String', function (msg) {
        if (msg instanceof LaserScan_1.LaserScan) {
            // Handle LaserScan message
            var data = JSON.parse(msg.data);
            handleData(data.topic, data.data);
        }
        else if (msg instanceof Imu_1.Imu) {
            // Handle Imu message
            var data = JSON.parse(msg.data);
            handleData(data.topic, data.data);
        }
        else if (typeof msg === 'string') {
            // Handle string messages
            console.log('Received string message:', msg);
        }
        else {
            // Handle other types
            console.error('Received unsupported message type:', msg);
        }
    });
    // Spin the node to handle messages
    rclnodejs.spin(node);
    // Cleanup on script termination
    process.on('exit', function () {
        handleExit();
    });
    function handleExit() {
        console.log("Received termination signal. Cleaning up...");
        // Additional cleanup logic if needed
        process.exit(0);
    }
});
