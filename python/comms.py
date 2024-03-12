import serial
import time
import threading

class ESP32:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.start = b'x'
        self.end = b'q'
        self.maxMessageLength = 256

    def check_req(self, req):
        #check if something is in serial buffer
        #check if it has start and endmarker, check for length
        #if yes, check with packets which type of packet it is
        #if packet type is known, send response
        #if not, delete buffer

        if self.ser.in_waiting > 0:
            req = self.ser.read(self.maxMessageLength)
            if req.startswith(self.start) and req.endswith(self.end):
                #split data in to type, nVal, access, data[nVal], error
                #data is split with ","
                #return data

                #split data looking for ","
                data = req.split(b",")

                packet_type = data[0]
                nVal = data[1]
                access = data[2]
                data = data[3]
                error = data[4]

                return packet_type, nVal, access, data, error







            else:
                return None


    def send_req(self, req):
        request = bytes(self.start + req + self.end, 'utf-8')
        self.ser.write(request)

if __name__ == "__main__":
    esp = ESP32()
    print("ready")
    while True:
        print("req!")
        esp.req_val()


class packets:
    def __init__(self):
        self.maxDataLength = 32
        self.data[maxDataLength]
        self.error

        #acsess: 0 read/write; 1 read only; 2 write only
        #error: 0 no error; 1 warning; 2 fatal error

        self.packet_types = {
            "isOk": {"type": 0, "nval": 0, "access": 0},
            "IMU1": {"type": 1, "nval": 6, "access": 1},
            "IMU2": {"type": 2, "nval": 3, "access": 1},
            "sonar": {"type": 3, "nval": 8, "access": 0},
            "voltage": {"type": 4, "nval": 5, "access": 0},
            "temperature": {"type": 6, "nval": 2, "access": 0},
            "led": {"type": 6, "nval": 3, "access": 0}

            # Add more packet types as needed
        }   
        
        #get ESP32 to use req and send val functions
        self.esp = ESP32()


    def formRequest(self, packet_type):
        request = str(packet_type) + "0" + str(access) + str(self.error)
        self.ESP32.send_req(request)

    def formResponse(self, packet_type, data):
        response = str(packet_type) + "1" + str(access) + str(self.error) + str(data)
        self.ESP32.send_req(response)

    def getRequest(self):
        self.ESO32.check_req()

        threading.Timer(1, self.getRequest).start()

if __name__ == "__main__":
    p = packets()
    IMU1 = p.formRequest("IMU1")
    print(IMU1)
    IMU2 = p.esp.send_req(p.formRequest("IMU2"))
    print(IMU2)

