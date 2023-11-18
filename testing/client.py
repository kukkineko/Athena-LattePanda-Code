import requests
import json
import subprocess
import time

class TypeScriptServer:
    def __init__(self):
        self.start_server()

    def start_server(self):
        self.server_process = subprocess.Popen(["node", "main.js"])
        time.sleep(2)  # Give the server some time to start

    def stop_server(self):
        requests.get("http://localhost:3000/shutdown")
        self.server_process.wait()  # Wait for the server process to complete

    def send_data(self, data):
        url = "http://localhost:3000/processData"
        data_to_send = {"data": data}
        response = requests.post(url, json=data_to_send)
        result = response.json()
        return result["result"]

    def __del__(self):
        self.stop_server()

# Example usage:
if __name__ == "__main__":
    # Create an instance of TypeScriptServer
    ts_server = TypeScriptServer()

    try:
        # Send data to the TypeScript server
        result = ts_server.send_data("Hello from Python!")
        print("Received result from TypeScript:", result)
    finally:
        # The server will be stopped when the instance is destroyed
        del ts_server
