import socket
import json
import subprocess
import signal

class CppClientInterface:
    def __init__(self, executable_path, host, port):
        self.executable_path = executable_path
        self.server_address = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(self.server_address)
        self.socket.listen(1)

    def start_cpp_app(self):
        if not hasattr(self, 'process') or self.process is None:
            self.process = subprocess.Popen([self.executable_path])
            signal.pause()  # Wait for a signal (e.g., SIGINT) to stop the subprocess

    def forward_data(self):
        connection, client_address = self.socket.accept()
        try:
            while True:
                data = connection.recv(4096)
                if data:
                    # Forward data to the C++ server
                    # Implement the logic to send data to the C++ server here
                    pass
        finally:
            connection.close()

    def __del__(self):
        if hasattr(self, 'process') and self.process is not None:
            self.process.send_signal(signal.SIGINT)
            self.process.wait()
        self.socket.close()

# Example usage
if __name__ == "__main__":
    cpp_client_interface = CppClientInterface("/path/to/cppclient", "localhost", 10000)

    try:
        cpp_client_interface.start_cpp_app()
        cpp_client_interface.forward_data()
    except KeyboardInterrupt:
        pass
