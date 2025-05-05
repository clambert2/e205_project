from ekf import EKF
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3
from adafruit_bus_device.i2c_device import I2CDevice
import board
import time
import threading
import numpy as np
import utils
from AlphaBot2 import AlphaBot2
from control import control_to_pose
from control import PoseThread
from lidar import get_scan
from rplidar import RPLidar


if __name__=='__main__':

    ab2 = AlphaBot2()
    # high_speed_thread = PoseThread(interval=0.01)
    # high_speed_thread.start()
    time.sleep(1)
    # print("pose: ", high_speed_thread.get_pose())

    # control_to_pose(high_speed_thread, ab2, 0.2, 0.2,
    #                 kp_lin=5, ki_lin=2,
    #                 kp_ang=10, ki_ang=2,
    #                 max_power=30, min_power=10,
    #                 pos_tolerance=0.06, ang_tolerance=0.2)

    # print("pose: ", high_speed_thread.get_pose())

    print(get_scan())
    # high_speed_thread.stop()
