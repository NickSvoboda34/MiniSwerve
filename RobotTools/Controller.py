import threading
import evdev
import math

class XboxController:
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        # ... (initialize other attributes as before)

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):
        # Return the buttons/triggers that you care about in this method
        x = self.LeftJoystickX
        y = self.LeftJoystickY
        a = self.A
        b = self.X
        rb = self.RightBumper
        return [x, y, a, b, rb]

    def _monitor_controller(self):
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if "Xbox" in device.name:
                xbox_device = device
                break
        else:
            raise Exception("Xbox controller not found!")

        for event in xbox_device.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                if event.code == evdev.ecodes.ABS_Y:
                    self.LeftJoystickY = event.value / XboxController.MAX_JOY_VAL
                elif event.code == evdev.ecodes.ABS_X:
                    self.LeftJoystickX = event.value / XboxController.MAX_JOY_VAL
                # ... (add other joystick mappings)
            elif event.type == evdev.ecodes.EV_KEY:
                if event.code == evdev.ecodes.BTN_SOUTH:
                    self.A = event.value
                elif event.code == evdev.ecodes.BTN_NORTH:
                    self.X = event.value
                # ... (add other button mappings)

# Example usage:
controller = XboxController()
while True:
    joystick_values = controller.read()
    print(f"Joystick values: X={joystick_values[0]}, Y={joystick_values[1]}, A={joystick_values[2]}, X={joystick_values[3]}, RB={joystick_values[4]}")
