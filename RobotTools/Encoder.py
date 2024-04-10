from threading import Thread
import smbus2 # type: ignore
import time
import math

class AS5600:
    def __init__(self, multiplexer_address, channel):
        """
        Initializes the AS5600 angle reader.

        :param i2c_bus: The I2C bus number (usually 1 for Raspberry Pi).
        :param multiplexer_address: The I2C address of the TCA9548A multiplexer.
        """
        self.bus = smbus2.SMBus(1)
        self.multiplexer_address = multiplexer_address
        self.encoder_address = 0x36
        self.lastPosition = None
        self.channel = channel
        self.loopTime = 0.02
        self.position = 0
        self.velocity = 0
        self.magnitude = 0

        self.monitorThread = Thread(target=self.monitorEncoder)
        self.monitorThread.daemon = True
        self.monitorThread.start()

    def select_multiplexer_channel(self, channel):
        """
        Selects the desired channel on the TCA9548A multiplexer.

        :param channel: The channel number (0 to 7).
        """
        self.bus.write_byte(self.multiplexer_address, 1 << channel)
    
    def monitorEncoder(self):
        while True:
            # Select the appropriate channel on the multiplexer
            self.select_multiplexer_channel(self.channel)    

            # Read the angle from the AS5600
            raw_data = self.bus.read_i2c_block_data(self.encoder_address, 0x0E, 2)
            self.position = ((raw_data[0]<<8) | raw_data[1]) / 4096 * math.pi * 2 - math.pi # Convert raw data to degrees
            
            if self.lastPosition == None: self.velocity = 0
            else: self.velocity = math.copysign((abs(self.position) - abs(self.lastPosition)) / self.loopTime, self.position - self.lastPosition)
            self.lastPosition = self.position

            raw_data = self.bus.read_i2c_block_data(self.encoder_address, 0x1B, 2)
            self.magnitude = (raw_data[0]<<8) | raw_data[1]

            time.sleep(self.loopTime)

    def getPosition(self):
        """
        Reads the angle from the AS5600 encoder.

        :return: Angle in radians (-pi to pi).
        """   

        return self.position

    def getVelocity(self):
        """
        Calculates the angular velocity (in radians per second).

        :return: Angular velocity.
        """
        return self.velocity

if __name__ == "__main__":
    # Example usage
    i2c_bus_number = 1
    tca9548a_address = 0x70  # Replace with your actual TCA9548A address

    encoder1 = AS5600(tca9548a_address, 0)
    encoder2 = AS5600(tca9548a_address, 1)
    encoder3 = AS5600(tca9548a_address, 2)

    while True:
        pos = encoder1.getPosition()
        vel = encoder1.getVelocity()
        print(f"Position: {pos:.2f}, Velocity: {vel:.2f}")

        #pos1 = encoder1.getPosition()
        #pos2 = encoder2.getPosition()
        #pos3 = encoder3.getPosition()
        #print(f"Current angle: {pos1:.2f}, {pos2:.2f}, {pos3:.2f}")
