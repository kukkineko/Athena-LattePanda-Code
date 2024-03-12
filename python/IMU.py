import serial

class IMU():
    def __init__(self):
        # Connect serial to /dev/ttyACM0 at 230400 baud
        self.ser = serial.Serial('/dev/ttyACM0', 230400)
        self.quaternion = [0.0, 0.0, 0.0, 0.0]
        self.avgGX, self.avgGY, self.avgGZ = 0.0, 0.0, 0.0
        self.avgAX, self.avgAY, self.avgAZ = 0.0, 0.0, 0.0

    def read(self):
        try:
            # Read the serial input
            data = self.ser.readline().decode().strip()

            # Check if "Quaternion:" is present in the data
            if "Quaternion:" in data:
                # Split the data into quaternion values
                quaternion_data = data.split("Quaternion:")[1].strip()
                # Extract the quaternion values
                self.quaternion = [float(q) for q in quaternion_data.split("\t")]
                # Print or process the third value of the quaternion
                print("Third value of quaternion:", self.quaternion[2])

            # Check if "Gyro:" is present in the data
            if "Gyro:" in data:
                # Split the data into gyro values
                gyro_data = data.split("Gyro:")[1].strip()
                # Extract the gyro values
                self.gx, self.gy, self.gz = gyro_data.split("\t")
                print(f"Gyro:\t{self.gx}\t{self.gy}\t{self.gz}")

            # Check if "Accel:" is present in the data
            if "Accel:" in data:
                # Split the data into accelerometer values
                accel_data = data.split("Accel:")[1].strip()
                # Extract the accelerometer values
                self.ax, self.ay, self.az = accel_data.split("\t")
                print(f"Accel:\t{self.ax}\t{self.ay}\t{self.az}")



        except Exception as e:
            print("Error:", e)

if __name__ == "__main__":  
    imu_instance = IMU()
    while True:
        imu_instance.read()
