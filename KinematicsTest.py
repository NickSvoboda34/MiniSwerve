from RobotTools import SwerveDriveKinematics
from RobotTools import Controller

kinematics = SwerveDriveKinematics.SwerveDriveKinematics([[-1, 1], [1, 1], [-1, -1], [1, -1]])
controller = Controller.XboxController()

while True:
    controllerValues = controller.read()
    controllerValues = Controller.applyDeadzone(controllerValues, 0.1)
    print(kinematics.toModuleStates([controllerValues[0], controllerValues[1], controllerValues[2]], [0, 0]))