from PIDController import PIDController
from MotorController import MotorController
from Encoder import Encoder
import math

class SwerveModule:
    wheelDiameter = 3

    def __init__(self, leftMotor:MotorController, rightMotor:MotorController, turnEncoder:Encoder, turnPID:PIDController, drivePID:PIDController):
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.turnEncoder = turnEncoder
        self.turnPID = turnPID
        self.drivePID = drivePID
    
    def setMotorSpeeds(self, leftSpeed:float, rightSpeed:float):
        self.leftMotor.setSpeed(leftSpeed)
        self.rightMotor.setSpeed(rightSpeed)

    def setModuleState(self, driveVelocity:float, angle:float):
        driveOutput = self.drivePID.calculate(driveVelocity, self.getModuleVelocity)
        turnOutput = self.turnPID.calculate(angle, self.turnEncoder.getPosition())
        self.setMotorSpeeds(driveOutput + turnOutput, -driveOutput + turnOutput)

    def getModuleVelocity(self):
        return (self.leftMotor.getVelocity() - self.rightMotor.getVelocity()) / (2 * math.pi) * self.wheelDiameter
   