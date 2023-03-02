import comms


esp = comms.ESP32()

while True:
    esp.req_val()