from RobotBase import RobotBase
from Encoder import Encoder

class MotorController(RobotBase):
    def __init__(self, speedPin, leftDirPin, rightDirPin, encoder:Encoder):
        self.speedPin = speedPin
        self.leftDirPin = leftDirPin
        self.rightDirPin = rightDirPin
        self.encoder = encoder

    def setPinVal(self, pin, value):
        self.ser.write(('0' * (2 - len(str(pin))) + str(pin) + ',' + '0' * (3 - len(str(value))) + str(value)).encode())
    
    def setSpeed(self, speed:float):
        if speed > 0:
            self.setPinVal(self.leftDirPin, 255)
            self.setPinVal(self.rightDirPin, 0)
        else:
            self.setPinVal(self.leftDirPin, 0)
            self.setPinVal(self.rightDirPin, 255)

        speed = (int) (abs(speed * 255))
        self.setPinVal(self.speedPin, speed)

    def getPosition(self):
        return self.encoder.getPosition()
    
    def getVelocity(self):
        return self.encoder.getVelocity()