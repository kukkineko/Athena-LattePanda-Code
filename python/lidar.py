import ydlidar
import time
import math
import serial


# Initialize the ydlidar module and connect to the laser rangefinder
# device via the serial port at /COM4.

class Lidar():

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
        self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, False);

        init =  self.laser.initialize()
        on =  self.laser.turnOn()
        
        if init and on:
            pass
        else:
            try:
                raise ValueError(self.laser.DescribeError())
            except ValueError as e:
                print(e)

        

    def getGrid(self):
        ls_pts_x = []
        ls_pts_y = []

        # Create a LaserScan object to hold the scan data.
        scan = ydlidar.LaserScan()

        # Attempt to get a scan from the device.
        if self.laser.doProcessSimple(scan):
            # Print the number of points in the scan and the scan frequency.
            #print(f"Scan received[{scan.stamp}]: {scan.points.size()} ranges is [{1.0 / scan.config.scan_time}]Hz")

            for n in range(0, len(scan.points)):
                angle = scan.points[n].angle
                dist = scan.points[n].range
                x, y = dist * math.cos(angle), dist * math.sin(angle)
                ls_pts_x.append((x*(-1)))
                ls_pts_y.append((y))
            #print(ls_pts)

            return ls_pts_x, ls_pts_y

        else:
            print("Failed to get Lidar Data")
            return [0], [0]

    def getAngleDist(self):
        ls_angle = []
        ls_range = []
        ls_intensity = []
        ls_scan_time = []
        ls_time_increment = []
        ls_angle_min = []
        ls_angle_max = []
        ls_angle_increment = []
        ls_range_min = []
        ls_range_max = []

        # Create a LaserScan object to hold the scan data.
        scan = ydlidar.LaserScan()

        # Attempt to get a scan from the device.
        if self.laser.doProcessSimple(scan):
            # Print the number of points in the scan and the scan frequency.
            #print(f"Scan received[{scan.stamp}]: {scan.points.size()} ranges is [{1.0 / scan.config.scan_time}]Hz")
            #send lidar info
            for n in range(0, len(scan.points)):
                #get lidar info
                angle = scan.points[n].angle
                range = scan.points[n].range
                intensity = scan.points[n].intensity
                scan_time = scan.config.scan_time
                time_increment = scan.config.time_increment
                angle_min = scan.config.min_angle
                angle_max = scan.config.max_angle
                angle_increment = scan.config.angle_increment
                range_min = scan.config.min_range
                range_max = scan.config.max_range

                #append to list
                ls_angle.append(angle)
                ls_range.append(range)
                ls_intensity.append(intensity)
            #print(ls_pts)

            return ls_angle, ls_range, ls_intensity, scan_time, time_increment, angle_min, angle_max, angle_increment, range_min, range_max

        else:
            print("Failed to get Lidar Data")
            return [0], [0]
        
        
    def turnOff(self):
        # Turn off the device.
        self.laser.turnOff()
        self.laser.disconnecting()

    def __del__(self):
        self.turnOff()
        print("Lidar turned off")

    


if __name__ == "__main__":
    lidar = Lidar()
    lidar.getGrid()
    lidar.getAngleDist()
    print(lidar.getGrid())

    
    