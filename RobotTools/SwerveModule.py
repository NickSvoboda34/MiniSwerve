from PIDController import PIDController
from MotorController import MotorController
from Encoder import AS5600
from RobotBase import RobotBase
import math

class SwerveModule:
    wheelDiameter = 3

    def __init__(self, leftMotor:MotorController, rightMotor:MotorController, turnEncoder:AS5600, turnPID:PIDController, drivePID:PIDController):
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.turnEncoder = turnEncoder
        self.turnPID = turnPID
        self.drivePID = drivePID
    
    def setMotorSpeeds(self, leftSpeed:float, rightSpeed:float):
        self.leftMotor.setSpeed(leftSpeed)
        self.rightMotor.setSpeed(rightSpeed)

    def setModuleState(self, driveVelocity:float, angle:float):
        driveOutput = self.drivePID.calculate(driveVelocity, self.getModuleVelocity())
        turnOutput = self.turnPID.calculate(angle, self.turnEncoder.getPosition())
        self.setMotorSpeeds(driveOutput + turnOutput, -driveOutput + turnOutput)

    def getModuleVelocity(self):
        return (self.leftMotor.getVelocity() - self.rightMotor.getVelocity()) / (2 * math.pi) * self.wheelDiameter

if __name__ == '__main__' :
    base = RobotBase('/dev/ttyACM0')

    leftMotor = MotorController(2, 24, 25, AS5600(0x70, 0))
    rightMotor = MotorController(3, 22, 23, AS5600(0x70, 2))
    encoder = AS5600(0x70, 1)
    turnPID = PIDController(1, 0, 0)
    drivePID = PIDController(1, 0, 0)
    module = SwerveModule(leftMotor, rightMotor, encoder, turnPID, drivePID)

    while True: 
        module.setModuleState(0, 0)