# client.py

import subprocess
import json
import sys
import rclnodejs
import atexit

class TypeScriptServerClient:
    def __init__(self):
        # Start TypeScript server
        self.ts_server_process = subprocess.Popen(
            ["node", "../nodejs/typescriptServer.js"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        # Wait for the server to start (you might want to add better handling)
        while True:
            line = self.ts_server_process.stdout.readline().strip()
            if line == 'TypeScript server is listening on port 3000':
                break

    def send_data(self, topic, data):
        # Send data to TypeScript server
        message = {"topic": topic, "data": data}
        message_str = json.dumps(message)
        self.ts_server_process.stdin.write(message_str + '\n')
        self.ts_server_process.stdin.flush()

    def close(self):
        # Close TypeScript server
        self.ts_server_process.terminate()
        self.ts_server_process.wait()

# Ensure the TypeScript server is closed when the Python script exits
def cleanup():
    ts_client.close()

atexit.register(cleanup)

if __name__ == '__main__':
    ts_client = TypeScriptServerClient()

    # Your existing code or any additional logic can go here

    # Example: sending data
    ts_client.send_data('imu', {'some': 'data'})

    # Close TypeScript server when the script exits
    ts_client.close()
