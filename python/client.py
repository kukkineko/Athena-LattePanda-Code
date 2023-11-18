import os
import requests
import json
import subprocess
import time

class TypeScriptServer:
    def __init__(self):
        # Get the absolute path to the directory of the current script
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # Build the path to the Node.js script
        node_script_path = os.path.join(script_dir, '..', 'nodejs', 'main.js')

        # Start the server
        self.start_server(node_script_path)

    def start_server(self, node_script_path):
        self.server_process = subprocess.Popen(["node", node_script_path])
        time.sleep(2)  # Give the server some time to start

    def stop_server(self):
        requests.get("http://localhost:3000/shutdown")
        self.server_process.wait()  # Wait for the server process to complete

    def send_data(self, message_type, data):
        payload = {"type": message_type, "data": data}
        url = "http://localhost:3000/processData"
        response = requests.post(url, json=payload)
        result = response.json()
        return result["result"]

    def __del__(self):
        self.stop_server()

# Example usage:
if __name__ == "__main__":
    # Create an instance of TypeScriptServer
    ts_server = TypeScriptServer()

    try:
        # Example Lidar and IMU data
        lidar_data = [1.2, 2.3, 3.4, 4.5]
        imu_data = [0.1, 0.2, 0.3]

        # Send Lidar data with message type "lidar"
        lidar_result = ts_server.send_data("lidar", lidar_data)
        print("Received result for Lidar data:", lidar_result)

        # Send IMU data with message type "imu"
        imu_result = ts_server.send_data("imu", imu_data)
        print("Received result for IMU data:", imu_result)
    finally:
        # The server will be stopped when the instance is destroyed
        del ts_server
