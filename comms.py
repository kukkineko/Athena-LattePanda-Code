import serial
import time


class ESP32:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

    def req_val(self):
        self.ser.write(b'xq')
        time.sleep(0.1)
        response = self.ser.readline().decode()
        print(response)
        if response.startswith("x") and response.endswith("q"):
            packet = response[1:-1]  # remove the start and end marker
            values = packet.split(",")  # split the packet into individual values
            sonar = values[:16]  # extract the first 16 values as "Sonar"
            imu = values[16:23]  # extract the next 7 values as "IMU"
            voltage = values[23:28]  # extract the next 5 values as "Voltage"
            temps = values[28:34] #extract the 6 values after voltage as "Temps"
            error = values[-1]  # last value is the error flag

            # cast the value as floats
            sonar = [float(x) for x in sonar]
            imu = [float(x) for x in imu]
            voltage = [float(x) for x in voltage]
            temps = [float(x) for x in temps]
            error = error == "1" # boolean flag for error
            
            # print out the result
            print("Sonar: ", sonar)
            print("IMU: ", imu)
            print("Voltage: ", voltage)
            print("Temps: ", temps)
            print("Error: ", error)

    def send_req(self, req):
        request = bytes("x" + str(req) + "q", "UTF-8")
        self.ser.write(request)

if __name__ == "__main__":
    esp = ESP32()
    print("ready")
    while True:
        print("req!")
        esp.req_val()