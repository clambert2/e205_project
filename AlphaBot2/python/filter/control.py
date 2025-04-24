from AlphaBot2 import AlphaBot2
import time
import numpy as np

class Control:

    def __init__(self, alpha_bot, imu, ekf):
        self.alpha_bot = alpha_bot
        self.imu = imu
        self.ekf = ekf
        
    def drive_forward(self, distance):
        """
        Drive the robot forward a specified distance.
        
        Args:
            distance (float): Distance to drive in meters.
        """
        wheel_circumference = 2 * np.pi * self.alpha_bot.wheel_radius
        wheel_rotations = distance / wheel_circumference
        self.alpha_bot.setMotor(1, 1)  # Set both motors to move forward
        time.sleep(wheel_rotations * (self.alpha_bot.wheel_base / self.alpha_bot.speed))  # Adjust sleep time based on speed
        self.alpha_bot.setMotor(0, 0)  # Stop the motors