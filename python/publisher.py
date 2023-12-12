from client import CppClientInterface, ClientSocket
from lidar import Lidar
from IMU import IMU
import json
import threading

class SensorPublisher:
    def __init__(self, host, port, cpp_executable_path):
        self.lidar = Lidar()
        self.imu = IMU()
        self.client_socket = ClientSocket(host, port)
        self.cpp_client = CppClientInterface(cpp_executable_path)
        self.cpp_client.start_cpp_app()

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


    def close(self):
        self.client_socket.close()
        self.cpp_client.stop_cpp_app()


def publish_sensor_data(sensor_publisher):
    try:
        while True:
            # Fetch and send LIDAR data
            lidar_data = sensor_publisher.gen_laser_scan_msg()
            if lidar_data:
                sensor_publisher.send_data({"type": "lidar", "data": lidar_data})
            
            # Fetch and send IMU data
            imu_data = sensor_publisher.gen_imu_msg()
            if imu_data:
                sensor_publisher.send_data({"type": "imu", "data": imu_data})
    except KeyboardInterrupt:
        pass

def main():
    host, port = "localhost", 10000
    cpp_executable_path = "/path/to/your/cppclient"
    sensor_publisher = SensorPublisher(host, port, cpp_executable_path)

    try:
        publish_sensor_data(sensor_publisher)
    finally:
        sensor_publisher.close()

if __name__ == "__main__":
    main()