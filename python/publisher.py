from client import TypeScriptServerClient
from lidar import Lidar
from IMU import IMU

class LaserScanPublisher:
    def __init__(self, ts_client):
        self.lidar = Lidar()
        self.ts_client = ts_client

    def publish_laser_scan(self):
        laser_scan_msg = self.gen_laser_scan_msg()
        if laser_scan_msg is not None:
            self.ts_client.send_data("lidar", laser_scan_msg)

    def gen_laser_scan_msg(self):
        scan = self.lidar.scan()
        if scan:
            laser_scan_msg = {
                "header": {
                    "stamp": {
                        "sec": 0,
                        "nanosec": 0,
                    },
                    "frame_id": 'scan',
                },
                "angle_min": self.lidar.angle_min,
                "angle_max": self.lidar.angle_max,
                "angle_increment": self.lidar.angle_increment,
                "time_increment": self.lidar.time_increment,
                "scan_time": self.lidar.scan_time,
                "range_min": self.lidar.range_min,
                "range_max": self.lidar.range_max,
                "ranges": self.lidar.ranges_list,
                "intensities": self.lidar.intensities_list,
            }

            return laser_scan_msg
        else:
            return None

class ImuPublisher:
    def __init__(self, ts_client):
        self.imu = IMU()
        self.ts_client = ts_client

    def publish_imu(self):
        self.imu.read()
        ax, ay, az = float(self.imu.ax), float(self.imu.ay), float(self.imu.az)
        gx, gy, gz = float(self.imu.gx), float(self.imu.gy), float(self.imu.gz)

        imu_msg = {
            "header": {
                "stamp": {
                    "sec": 0,
                    "nanosec": 0,
                },
                "frame_id": 'imu',
            },
            "orientation": {"x": -1.0, "y": -1.0, "z": -1.0, "w": -1.0},
            "angular_velocity": {"x": gx, "y": gy, "z": gz},
            "linear_acceleration": {"x": ax, "y": ay, "z": az},
        }

        self.ts_client.send_data("imu", imu_msg)

def lidar_publisher_process():
    ts_client = TypeScriptServerClient()
    laser_scan_publisher = LaserScanPublisher(ts_client)

    try:
        while True:
            laser_scan_publisher.publish_laser_scan()

    except KeyboardInterrupt:
        pass

def imu_publisher_process():
    ts_client = TypeScriptServerClient()
    imu_publisher = ImuPublisher(ts_client)

    try:
        while True:
            imu_publisher.publish_imu()

    except KeyboardInterrupt:
        pass

def main():
    try:
        lidar_publisher_process()
        imu_publisher_process()

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
