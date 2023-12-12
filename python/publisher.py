from client import ClientSocket  # Assuming ClientSocket is defined in client.py
from lidar import Lidar
from IMU import IMU
import json
import threading

class SensorPublisher:
    def __init__(self, host, port):
        self.lidar = Lidar()
        self.imu = IMU()
        self.client_socket = ClientSocket(host, port)

    def send_data(self, data):
        try:
            message = json.dumps(data).encode()
            self.client_socket.send_data(message)
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


def publish_lidar_data(sensor_publisher):
    try:
        while True:
            lidar_data = sensor_publisher.gen_laser_scan_msg()
            if lidar_data:
                sensor_publisher.send_data({"type": "lidar", "data": lidar_data})
    except KeyboardInterrupt:
        pass

def publish_imu_data(sensor_publisher):
    try:
        while True:
            imu_data = sensor_publisher.gen_imu_msg()
            if imu_data:
                sensor_publisher.send_data({"type": "imu", "data": imu_data})
    except KeyboardInterrupt:
        pass

def main():
    host, port = "localhost", 10000
    sensor_publisher = SensorPublisher(host, port)

    lidar_thread = threading.Thread(target=publish_lidar_data, args=(sensor_publisher,))
    imu_thread = threading.Thread(target=publish_imu_data, args=(sensor_publisher,))

    lidar_thread.start()
    imu_thread.start()

    lidar_thread.join()
    imu_thread.join()

if __name__ == "__main__":
    main()