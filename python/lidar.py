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

    def __del__(self):
        self.turnOff()
        print("Lidar turned off")
        

    def getGrid(self): #"legacy" command, do not use anymore
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
        
    def scan(self):
        scan = ydlidar.LaserScan()

        #try to scan
        if self.laser.doProcessSimple(scan):
            for n in range(0, len(scan.points)):
                self.angle = scan.points[n].angle
                self.angle_min = scan.config.min_angle
                self.angle_max = scan.config.max_angle
                self.angle_increment = scan.config.angle_increment
                self.range = scan.points[n].range
                self.range_min = scan.config.min_range
                self.range_max = scan.config.max_range
                self.intensity = scan.points[n].intensity
                self.time_increment = scan.config.time_increment
                self.scan_time = scan.config.scan_time
                self.scan_frequency = scan.config.scan_frequency
                self.grid = scan.points
                self.grid_size = scan.points.size()
                self.grid_time = scan.stamp
                self.grid_health = scan.health
                self.grid_descriptor = scan.descriptor
                self.grid_type = scan.type
                self.grid_size = scan.points.size()

            return True    #return True if scan is successful
        else:
            return False    #return False if scan is unsuccessful


    def turnOff(self):
        # Turn off the device.
        self.laser.turnOff()
        self.laser.disconnecting()



    


if __name__ == "__main__":
    lidar = Lidar()
    lidar.getGrid()
    lidar.getAngleDist()
    print(lidar.getGrid())

    
    