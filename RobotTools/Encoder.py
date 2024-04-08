import smbus2
import time
import math

class Encoder:
    def __init__(self, i2c_bus, multiplexer_address):
        """
        Initializes the AS5600 angle reader.

        :param i2c_bus: The I2C bus number (usually 1 for Raspberry Pi).
        :param multiplexer_address: The I2C address of the TCA9548A multiplexer.
        """
        self.bus = smbus2.SMBus(i2c_bus)
        self.multiplexer_address = multiplexer_address
        self.encoder_address = 0x36
        self.last_angle = None
        self.last_timestamp = None

    def select_multiplexer_channel(self, channel):
        """
        Selects the desired channel on the TCA9548A multiplexer.

        :param channel: The channel number (0 to 7).
        """
        self.bus.write_byte(self.multiplexer_address, 1 << channel)

    def getPosition(self):
        """
        Reads the angle from the AS5600 encoder.

        :return: Angle in radians (0 to 2*pi).
        """
        # Select the appropriate channel on the multiplexer
        self.select_multiplexer_channel(0)  # Replace with the desired channel

        # Read the angle from the AS5600
        raw_data = self.bus.read_word_data(self.encoder_address, 0x0E)
        angle_degrees = (raw_data & 0x0FFF) * 0.08789  # Convert raw data to degrees
        angle_radians = math.radians(angle_degrees)

        return angle_radians

    def getVelocity(self):
        """
        Calculates the angular velocity (in radians per second).

        :return: Angular velocity.
        """
        current_angle = self.read_angle()
        current_timestamp = time.time()

        if self.last_angle is None or self.last_timestamp is None:
            # Initialize the last angle and timestamp
            self.last_angle = current_angle
            self.last_timestamp = current_timestamp
            return 0.0

        # Calculate change in angle and time
        delta_angle = current_angle - self.last_angle
        delta_time = current_timestamp - self.last_timestamp

        # Calculate angular velocity
        angular_velocity = delta_angle / delta_time

        # Update last angle and timestamp
        self.last_angle = current_angle
        self.last_timestamp = current_timestamp

        return angular_velocity

if __name__ == "__main__":
    # Example usage
    i2c_bus_number = 1
    tca9548a_address = 0x70  # Replace with your actual TCA9548A address

    angle_reader = Encoder(i2c_bus_number, tca9548a_address)
    current_angle = angle_reader.getPosition()
    print(f"Current angle: {current_angle:.2f} radians")

    angular_velocity = angle_reader.getVelocity()
    print(f"Angular velocity: {angular_velocity:.2f} radians per second")
