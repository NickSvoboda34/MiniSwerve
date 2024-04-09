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
            raw_data = self.bus.read_word_data(self.encoder_address, 0x0E)
            angle_degrees = (raw_data & 0x0FFF) * 0.08789  # Convert raw data to degrees
            self.position = math.radians(angle_degrees)

            if self.lastPosition == None: self.velocity = 0
            else: self.velocity = (self.position - self.lastPosition) / self.loopTime
            self.lastPosition = self.position

            time.sleep(self.loopTime)

    def getPosition(self):
        """
        Reads the angle from the AS5600 encoder.

        :return: Angle in radians (0 to 2*pi).
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

    angle_reader = AS5600(i2c_bus_number, tca9548a_address)
    current_angle = angle_reader.getPosition()
    print(f"Current angle: {current_angle:.2f} radians")

    angular_velocity = angle_reader.getVelocity()
    print(f"Angular velocity: {angular_velocity:.2f} radians per second")
