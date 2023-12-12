import subprocess
import json
import sys
import atexit
import time
import signal
import logging
import os

class CPPClient:
    def __init__(self):
        try:
            # Start CPP Client
            self.cpp_client_process = subprocess.Popen(
                ["./path_to_compiled_cppclient/cppclient"],  # Adjust the path to your compiled cppclient
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid  # Create a new process group
            )
        except Exception as e:
            logging.error(f"Error starting CPP Client: {e}")
            sys.exit(1)

        # Wait for the CPP client to start with a timeout
        max_retries = 10
        retry_interval = 1  # in seconds

        for _ in range(max_retries):
            if self.cpp_client_process.poll() is not None:
                # Process has exited, no need to wait
                break
            time.sleep(retry_interval)
        else:
            logging.error("Failed to start CPP Client within the given timeout.")
            sys.exit(1)

        # Register cleanup function
        atexit.register(self.close)

        # Register signal handler for termination signals
        signal.signal(signal.SIGINT, self.handle_exit)
        signal.signal(signal.SIGTERM, self.handle_exit)

    def send_data(self, topic, data):
        # This method may be redundant if your CPP client does not accept input via stdin.
        # Send data to CPP Client
        message = {"topic": topic, "data": data}
        message_str = json.dumps(message)
        self.cpp_client_process.stdin.write(message_str + '\n')
        self.cpp_client_process.stdin.flush()

    def close(self):
        # Close CPP Client
        if hasattr(self, 'cpp_client_process') and self.cpp_client_process.poll() is None:
            os.killpg(os.getpgid(self.cpp_client_process.pid), signal.SIGTERM)
            self.cpp_client_process.wait()
            logging.info("CPP Client terminated.")

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
    with CPPClient() as cpp_client:
        # Your existing code or any additional logic can go here
        # Example: cpp_client.send_data('imu', {'some': 'data'})
