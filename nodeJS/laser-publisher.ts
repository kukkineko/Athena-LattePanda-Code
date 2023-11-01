'use strict';

import * as rclnodejs from 'rclnodejs';
import * as rp from 'request-promise';
import { spawn } from 'child_process';
import * as path from 'path';

/**
 * ROS2 publisher of simulated LaserScan messages.
 * A timer is used to create and publish a laserScan message once per second (1 Hz).
 * Use start() and stop() to initiate and terminate messgage publication.  
 */
export class LaserScanPublisher {

    private isRunning = false;
    private publisher: rclnodejs.Publisher;
    private publisherTimer: rclnodejs.Timer;

    /**
     * Create an instance
     * 
     * @param node - The node that to which this publisher belongs.
     * @param topic - The topic name to which laserScan msgs will be published.
     */
    constructor(public readonly node: rclnodejs.Node, public readonly topic = 'laser_frame') {
        this.publisher = node.createPublisher('sensor_msgs/msg/LaserScan', topic);

        // Start the Python server on initialization
        const py = spawn('python', [path.join(__dirname, '..', 'python', 'server.py')]);
        py.stdout.on('data', function (data) {
            console.log('Python server output:', data.toString());
        });
    }

    /**
     * Start the laserScan message generation and publishing process.
     * 
     * @param interval - The unit of time (milliseconds) to wait before running the next .
     */
    start(interval = 1000): void {
        if (this.isRunning) return;

        this.isRunning = true;

        this.publisherTimer = this.node.createTimer(interval, async () => {
            let msg = await this.genLaserScanMsg();
            console.log('publish msg at ', msg.header.stamp.sec);
            this.publisher.publish(msg);
        });
    }

    /**
     * Stop creating and publishing laserScan messages.
     */
    stop() {
        this.publisherTimer.cancel();
        this.publisherTimer = null;
        this.isRunning = false;
    }

    /**
     * Creates a simulated forward facing LaserScan message.
     * Scan data consists of 180 measurements on a 180 degree arc centered on the
     * x-axis extending directly forward of the virutal lidar device.
     * 
     * @param range  - The distance of simulated lidar range readings
     * @returns A new LaserScan message
     */
    protected async genLaserScanMsg(range = 10): Promise<rclnodejs.sensor_msgs.msg.LaserScan> {

        // get data from internal server
        const options = {
            uri: 'http://localhost:8000/data',
            json: true
        };
        const data = await rp(options);

        // create empty laserScan msg
        let laserScanMsg  = rclnodejs.createMessageObject('sensor_msgs/msg/LaserScan') as 
            rclnodejs.sensor_msgs.msg.LaserScan;

        // configure msg header 
        laserScanMsg.header.frame_id = 'laser_frame';
        laserScanMsg.header.stamp = this.node.now().toMsg();

        // configure msg data
        laserScanMsg.angle_min = 0;
        laserScanMsg.angle_max = Math.PI / 2.0;
        laserScanMsg.angle_increment = Math.PI / 180.0;
        laserScanMsg.time_increment = 1.0 / data.array.length;
        laserScanMsg.scan_time = 1.0;
        laserScanMsg.range_min = range - 1;
        laserScanMsg.range_max = range + 1;
        laserScanMsg.ranges = data.array;

        return laserScanMsg;
    }
}
