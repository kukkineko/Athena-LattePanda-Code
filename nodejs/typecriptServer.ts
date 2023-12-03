'strict mode';

import * as rclnodejs from 'rclnodejs';

/**
 * ROS2 publisher for simulated sensor data.
 */

export class SensorPublisher {

    private isRunning = false;
    private lidarPublisher: rclnodejs.Publisher;
    private imuPublisher: rclnodejs.Publisher;
    private publisherTimer: rclnodejs.Timer;

    /**
     * Create an instance
     * 
     * @param node - The node that to which this publisher belongs.
     */
    constructor(public readonly node: rclnodejs.Node) {
        this.lidarPublisher = node.createPublisher('sensor_msgs/msg/LaserScan', 'laser_frame');
        this.imuPublisher = node.createPublisher('sensor_msgs/msg/Imu', 'imu_frame');
    }

    /**
     * Start the sensor data generation and publishing process.
     * 
     * @param interval - The unit of time (milliseconds) to wait before running the next .
     */
    start(interval = 1000): void {
        if (this.isRunning) return;

        this.isRunning = true;

        this.publisherTimer = this.node.createTimer(interval, () => {
            const laserScanMsg = this.genLaserScanMsg();
            const imuMsg = this.genImuMsg();

            console.log('Publishing LaserScan and IMU data');
            this.lidarPublisher.publish(laserScanMsg);
            this.imuPublisher.publish(imuMsg);
        });
    }

    /**
     * Stop creating and publishing sensor data.
     */
    stop() {
        if (!this.isRunning) return;

        this.publisherTimer.cancel();
        this.publisherTimer = null;
        this.isRunning = false;
    }

    /**
     * Creates a simulated forward facing LaserScan message.
     * @returns A new LaserScan message
     */
    protected genLaserScanMsg(): rclnodejs.sensor_msgs.msg.LaserScan {
        // Similar implementation to your genLaserScanMsg
    }

    /**
     * Creates a simulated IMU message.
     * @returns A new IMU message
     */
    protected genImuMsg(): rclnodejs.sensor_msgs.msg.Imu {
        // Implement your logic to generate IMU data
    }
}

async function main() {
    rclnodejs.init().then(() => {
        const node = rclnodejs.createNode('sensor_publisher_node');
        const sensorPublisher = new SensorPublisher(node);

        sensorPublisher.start(1000); // Start publishing sensor data every 1000 milliseconds

        // Handle termination
        process.on('SIGINT', () => {
            sensorPublisher.stop();
            rclnodejs.shutdown();
        });
    });
}

main();
