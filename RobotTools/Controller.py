from continuous_threading import ContinuousThread
import evdev
import math

class XboxController:
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        # Add other attributes here (e.g., RightJoystickY, RightJoystickX, etc.)

        self.A = 0  # Initialize the A button attribute
        self.X = 0  # Initialize the X button attribute
        # Add other button attributes here (e.g., B, Y, RB, etc.)

        self._monitor_thread = ContinuousThread(target=self._monitor_controller)
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
                # Add other joystick mappings here
            elif event.type == evdev.ecodes.EV_KEY:
                if event.code == evdev.ecodes.BTN_SOUTH:  # A button
                    self.A = event.value
                elif event.code == evdev.ecodes.BTN_NORTH:  # X button
                    self.X = event.value
                # Add other button mappings here

# Example usage:
controller = XboxController()
while True:
    joystick_values = controller.read()
    print(f"Joystick values: X={joystick_values[0]}, Y={joystick_values[1]}, A={joystick_values[2]}, X={joystick_values[3]}, RB={joystick_values[4]}")
