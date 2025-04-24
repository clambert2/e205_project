from imu import IMU
import time
import numpy as np
import matplotlib.pyplot as plt
from AlphaBot2 import AlphaBot2


# Write some imu test code that prints to terminal the values of the imu and integrated values

if __name__=='__main__':
    imu = IMU(interval=0.02)
    time.sleep(1)

    # Then perform test when the robot performs a movement
    # Initialize the robot
    alpha_bot = AlphaBot2()
    alpha_bot.setPWMA(20)
    alpha_bot.setPWMB(20)

    # Clear the IMU data
    imu.clear_data()
    time.sleep(0.2)
    alpha_bot.setMotor(15,-15)
    time.sleep(1)
    alpha_bot.stop()
    time.sleep(1)

    print(imu.update_pose())
