import serial

class IMU():
    def __init__(self):
        # Connect serial to /dev/ttyACM0 at 230400 baud
        self.ser = serial.Serial('/dev/ttyACM0', 230400)
        self.ax, self.ay, self.az = 0.0, 0.0, 0.0
        self.gx, self.gy, self.gz = 0.0, 0.0, 0.0

    def read(self):
        # Read the serial input
        data = self.ser.readline().decode().strip()

        # Check if "gyro:" is present in the data
        if "gyro:" in data:
            # Split the data into gyro values
            gyro_data = data.split("gyro:")[1].strip()
            # Extract the gyro values
            self.gx, self.gy, self.gz = gyro_data.split("\t")
            # Print or process gyro values as needed
            print(f"Gyro:\t{self.gx}\t{self.gy}\t{self.gz}")

        # Check if "accel:" is present in the data
        if "accel:" in data:
            # Split the data into accel values
            accel_data = data.split("accel:")[1].strip()
            # Extract the accel values
            self.ax, self.ay, self.az = accel_data.split("\t")
            # Print or process accel values as needed
            print(f"Accel:\t{self.ax}\t{self.ay}\t{self.az}")

if __name__ == "__main__":
    imu_instance = IMU()
    while True:
        imu_instance.read()
