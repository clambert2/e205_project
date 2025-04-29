from imu import IMU
from ekf import EKF
import time
from AlphaBot2 import AlphaBot2


# Write some imu test code that prints to terminal the values of the imu and integrated values

if __name__=='__main__':
    # Initialize the IMU
    imu = IMU(interval=0.01)
    ekf = EKF()
    time.sleep(1)

    # Initialize the robot
    alpha_bot = AlphaBot2()

    # Set motor speed
    alpha_bot.setPWMA(20)
    alpha_bot.setPWMB(20)

    # Clear the IMU data
    imu.clear_data()
    time.sleep(0.2)
    alpha_bot.setMotor(15,-15)
    time.sleep(0.5)
    alpha_bot.stop()
    time.sleep(1)

    print(ekf.prediction(imu.get_data(), is_turn=True))
