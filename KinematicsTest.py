from RobotTools import SwerveDriveKinematics
from evdev import InputDevice, categorize, ecodes

kinematics = SwerveDriveKinematics.SwerveDriveKinematics([[-6, 6], [6, 6], [-6, -6], [6, -6]])

print(kinematics.toModuleStates([1, 1, 0], [0, 0]))

controller = InputDevice('/dev/input/event8')

lx, ly, rx = 0

while True:
    for event in controller.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == 0:
                lx = (int) (abs(1 - event.value / 32767) * 255)
            if event.code == 1:
                ly = (int) (abs(1 - event.value / 32767) * 255)
            if event.code == 2:
                rx = (int) (abs(1 - event.value / 32767) * 255)

            speeds = [lx, ly, rx]
            print('\n' * 10)
            print(kinematics.toModuleStates(speeds, [0, 0]))