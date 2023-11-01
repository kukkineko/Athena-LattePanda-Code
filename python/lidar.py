import ydlidar
import time
import math
import serial

class Lidar:
    def __init__(self):
        ydlidar.os_init()
        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, "/dev/ttyUSB0")
        # Set the baud rate and other configuration options for the device.
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 5)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

        init =  self.laser.initialize()
        on =  self.laser.turnOn()

        if init and on:
            self.ranges_list = []
            self.intensities_list = []
        else:
            try:
                raise ValueError(self.laser.DescribeError())
            except ValueError as e:
                print(e)

    def __del__(self):
        self.turnOff()
        print("Lidar turned off")

    def scan(self):
        scan = ydlidar.LaserScan()

        # Try to scan
        if self.laser.doProcessSimple(scan):
            self.ranges_list = []  # Clear the ranges list
            self.intensities_list = []  # Clear the intensities list
            self.angle = scan.points[n].angle
            self.angle_min = scan.config.min_angle
            self.angle_max = scan.config.max_angle
            self.angle_increment = scan.config.angle_increment
            self.range_min = scan.config.min_range
            self.range_max = scan.config.max_range
            self.time_increment = scan.config.time_increment
            self.scan_time = scan.config.scan_time
            self.grid = scan.points
            self.grid_size = scan.points.size()
            self.grid_time = scan.stamp
            self.grid_size = scan.points.size()

            for n in range(0, len(scan.points)):
                self.range = scan.points[n].range
                self.intensity = scan.points[n].intensity
         
                # Append the range and intensity to the respective lists
                self.ranges_list.append(self.range)
                self.intensities_list.append(self.intensity)

            return True  # Return True if scan is successful
        else:
            return False  # Return False if scan is unsuccessful

    def turnOff(self):
        # Turn off the device.
        self.laser.turnOff()
        self.laser.disconnecting()
