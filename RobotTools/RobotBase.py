import serial, time as t

class RobotBase:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=.02)
    #Port usually /dev/ttyACM0
    def __init__(self, serialPort):
        self.ser = serial.Serial(serialPort, 115200, timeout=0.02)
        self.ser.reset_input_buffer()
        t.sleep(1)