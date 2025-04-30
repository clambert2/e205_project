from ekf import EKF
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3
from adafruit_bus_device.i2c_device import I2CDevice
import board
import time
import threading
import numpy as np


class PoseThread(threading.Thread):
    def __init__(self, interval=0.01):
        super().__init__()
        self.is_still = True
        self.is_turn = False
        self.is_move = False
        self.interval = interval
        self.running = True
        self.lock = threading.Lock()

        # SETUP IMU HERE
        self.i2c = board.I2C()
        self.imu = LSM6DS3(self.i2c)
        self.last_time = time.time()
        self.ekf = EKF()
        self.u = np.zeros((2, 1))

        with I2CDevice(self.i2c, 0x6A) as device:
            # ACCEL: ODR = 104Hz (0b0100), FS = ±2g (0b00) => 0b01000000 = 0x40
            device.write(bytes([0x10, 0x40]))

            # GYRO: ODR = 104Hz (0b0100), FS = ±1000 dps (0b11) => 0b01001100 = 0x4C
            device.write(bytes([0x11, 0x4C]))

    def run(self):
        while self.running:
            self.u = np.zeros((2, 1))
            with self.lock:
                if self.is_move:
                    self.u[0, 0] = self.imu.acceleration[1]
                elif self.is_turn:
                    self.u[1, 0] = self.imu.gyro[2] * 8  # NOTE CONSTANT ADDED FOUND IN SCALING
            timestamp = time.time()
            dt = timestamp - self.last_time
            self.last_time = timestamp
            if not self.is_still:
                self.ekf.prediction(self.u, dt)
            #print("Pose: ", self.pose)
            time.sleep(self.interval)

    def robot_still(self):
        with self.lock:
            self.is_still = True
            self.is_turn = False
            self.is_move = False

    def robot_move(self):
        with self.lock:
            self.is_still = False
            self.is_turn = False
            self.is_move = True

    def robot_turn(self):
        with self.lock:
            self.is_still = False
            self.is_turn = True
            self.is_move = False

    def get_pose(self):
        with self.lock:
            return self.ekf.x_bar
    
    def get_pose_and_u(self):
        with self.lock:
            return self.ekf.x_bar, self.u

    def stop(self):
        self.running = False
        self.join()



if __name__=='__main__':
    # Initialize the robot components
    high_speed_thread = PoseThread()
    high_speed_thread.start()

    time.sleep(2)
    print("pose: ", high_speed_thread.get_pose_and_u())
    high_speed_thread.robot_move()
    time.sleep(2)
    print("pose: ", high_speed_thread.get_pose_and_u())
    high_speed_thread.robot_still()
    time.sleep(2)
    print("pose: ", high_speed_thread.get_pose_and_u())
    high_speed_thread.robot_turn()
    time.sleep(5)
    print("pose: ", high_speed_thread.get_pose_and_u())
    high_speed_thread.robot_move()
    time.sleep(2)
    print("pose: ", high_speed_thread.get_pose_and_u())
    high_speed_thread.stop()