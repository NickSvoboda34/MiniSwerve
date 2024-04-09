from threading import Thread
import evdev # type: ignore
import math
import time

def applyDeadzone(values, deadzone):
    return [x if abs(x) > deadzone else 0 for x in values]

class XboxController:
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickX = 0
        # Add other attributes here (e.g., RightJoystickY, RightJoystickX, etc.)

        self._monitor_thread = Thread(target=self._monitor_controller)
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):
        # Return the buttons/triggers that you care about in this method
        lx = self.LeftJoystickX
        ly = self.LeftJoystickY
        rx = self.RightJoystickX
        return [lx, ly, rx]

    def _monitor_controller(self):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if "Xbox Wireless Controller" == device.name:
                xbox_device = device
                break
        else:
            raise Exception("Xbox controller not found!")

        print(xbox_device.read_loop())

        for event in xbox_device.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                if event.code == evdev.ecodes.ABS_Y:
                    self.LeftJoystickY = event.value / XboxController.MAX_JOY_VAL - 1
                elif event.code == evdev.ecodes.ABS_X:
                    self.LeftJoystickX = event.value / XboxController.MAX_JOY_VAL - 1
                elif event.code == 2:
                    self.RightJoystickX = event.value / XboxController.MAX_JOY_VAL - 1

# Example usage:
if __name__ == '__main__':
    controller = XboxController()
    while True:
        time.sleep(0.02)
        joystick_values = controller.read()
        joystick_values = applyDeadzone(joystick_values, 0.1)
        print(f"Joystick values: LX={joystick_values[0]:0.2f}, LY={joystick_values[1]:0.2f}, RX={joystick_values[2]:0.2f}")
