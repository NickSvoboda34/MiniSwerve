import serial, time
from evdev import InputDevice, categorize, ecodes

if __name__ == '__main__':
    controller = InputDevice('/dev/input/event8')
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.02)
    ser.reset_input_buffer()

    time.sleep(1)
    ser.write(b'25,255')

    while True:
        for event in controller.read_loop():
            if event.type == ecodes.EV_ABS:
                if event.code == 0:
                    val = (int) (abs(1 - event.value / 32767) * 255)
                    send = '02,' + '0' * (3 - len(str(val))) + str(val)
                    ser.write(send.encode())
                    print(ser.readline(6))

'''
    startTime = time.time()
    ser.write(b'13,255')
    print(ser.readline(6))
    print(time.time() - startTime)
'''