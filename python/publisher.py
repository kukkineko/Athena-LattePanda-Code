from lidar import Lidar
from IMU import IMU
import socket
import json

class SensorPublisher:
    def __init__(self, host, port):
        self.lidar = Lidar()
        self.imu = IMU()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = (host, port)
        self.socket.connect(self.server_address)

    def send_data(self, data):
        try:
            message = json.dumps(data).encode()
            self.socket.sendall(message)
        except Exception as e:
            print(f"Error sending data: {e}")

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

    def gen_imu_msg(self):
        self.imu.read()
        imu_msg = {
            "header": {
                "stamp": {
                    "sec": 0,
                    "nanosec": 0,
                },
                "frame_id": 'imu',
            },
            "orientation": {"x": -1.0, "y": -1.0, "z": -1.0, "w": -1.0},
            "angular_velocity": {"x": float(self.imu.gx), "y": float(self.imu.gy), "z": float(self.imu.gz)},
            "linear_acceleration": {"x": float(self.imu.ax), "y": float(self.imu.ay), "z": float(self.imu.az)},
        }
        return imu_msg

def lidar_publisher_process(host, port):
    sensor_publisher = SensorPublisher(host, port)
    try:
        while True:
            lidar_data = sensor_publisher.gen_laser_scan_msg()
            if lidar_data:
                sensor_publisher.send_data({"type": "lidar", "data": lidar_data})
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.socket.close()

def imu_publisher_process(host, port):
    sensor_publisher = SensorPublisher(host, port)
    try:
        while True:
            imu_data = sensor_publisher.gen_imu_msg()
            if imu_data:
                sensor_publisher.send_data({"type": "imu", "data": imu_data})
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.socket.close()

def main():
    host, port = "localhost", 10000  # Set the appropriate host and port
    try:
        # Parallel processing will be required here for simultaneous execution
        lidar_publisher_process(host, port)
        imu_publisher_process(host, port)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
