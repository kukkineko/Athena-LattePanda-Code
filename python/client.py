import socket
import subprocess

class CppClientInterface:
    def __init__(self, executable_path):
        self.executable_path = executable_path
        self.process = None

    def start_cpp_app(self):
        if self.process is None:
            self.process = subprocess.Popen([self.executable_path])
            print("C++ application started.")

    def stop_cpp_app(self):
        if self.process is not None:
            self.process.terminate()
            self.process.wait()
            print("C++ application stopped.")

    def __del__(self):
        self.stop_cpp_app()

class ClientSocket:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))

    def send_data(self, data):
        self.socket.sendall(data)

    def close(self):
        self.socket.close()
