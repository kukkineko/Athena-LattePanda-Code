import subprocess
import json
import sys
import atexit
import time
import signal
import logging
import os

class TypeScriptServerClient:
    def __init__(self):
        try:
            # Start TypeScript server
            self.ts_server_process = subprocess.Popen(
                ["node", "../nodejs/typescriptServer.js"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid  # Create a new process group
            )
        except Exception as e:
            logging.error(f"Error starting TypeScript server: {e}")
            sys.exit(1)

        # Wait for the server to start with a timeout
        max_retries = 10
        retry_interval = 1  # in seconds

        for _ in range(max_retries):
            line = self.ts_server_process.stdout.readline().strip()
            if line == 'TypeScript server is listening on port 3000':
                break
            time.sleep(retry_interval)
        else:
            logging.error("Failed to start TypeScript server within the given timeout.")
            sys.exit(1)

        # Register cleanup function
        atexit.register(self.close)

        # Register signal handler for termination signals
        signal.signal(signal.SIGINT, self.handle_exit)
        signal.signal(signal.SIGTERM, self.handle_exit)

    def send_data(self, topic, data):
        # Send data to TypeScript server
        message = {"topic": topic, "data": data}
        message_str = json.dumps(message)
        self.ts_server_process.stdin.write(message_str + '\n')
        self.ts_server_process.stdin.flush()

    def close(self):
        # Close TypeScript server
        if hasattr(self, 'ts_server_process') and self.ts_server_process.poll() is None:
            os.killpg(os.getpgid(self.ts_server_process.pid), signal.SIGTERM)
            self.ts_server_process.wait()
            logging.info("TypeScript server terminated.")

    def handle_exit(self, signum, frame):
        # Handle termination signals
        logging.info("Received termination signal. Cleaning up...")
        self.close()
        sys.exit(0)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

# Configure logging
logging.basicConfig(level=logging.INFO)

if __name__ == '__main__':
    with TypeScriptServerClient() as ts_client:
        # Your existing code or any additional logic can go here
        ts_client.send_data('imu', {'some': 'data'})
