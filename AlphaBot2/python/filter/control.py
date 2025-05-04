from AlphaBot2 import AlphaBot2
from ekf import EKF
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3
from adafruit_bus_device.i2c_device import I2CDevice
import board
import time
import threading
import numpy as np
import utils


class PoseThread(threading.Thread):
    def __init__(self, interval=0.005):
        super().__init__()
        self.is_still = True
        self.is_turn = False
        self.is_move = False
        self.interval = interval
        self.running = True
        self.lock = threading.Lock()
        self.alpha = 0.4

        # SETUP IMU HERE
        self.i2c = board.I2C()
        self.imu = LSM6DS3(self.i2c)
        self.ekf = EKF()
        self.u = np.zeros((2, 1))
        self.u_prev = np.zeros((2, 1))
        self.last_time = time.time()

        with I2CDevice(self.i2c, 0x6A) as device:
            # ACCEL: ODR = 104Hz (0b0100), FS = ±2g (0b00) => 0b01000000 = 0x40
            device.write(bytes([0x10, 0x40]))

            # GYRO: ODR = 104Hz (0b0100), FS = ±1000 dps (0b11) => 0b01001100 = 0x4C
            device.write(bytes([0x11, 0x4C]))

        # Zero the imu a values
        still_meas = np.mean([self.imu.acceleration for _ in range(100)], axis=0)
        expected_meas = np.array([0, 0, 9.81])
        self.imu_calibration = utils.rotation_matrix_from_vectors(still_meas, expected_meas)


    def run(self):
        self.last_time = time.time()
        while self.running:
            self.u = np.zeros((2, 1))
            with self.lock:
                if self.is_move:
                    self.u[0, 0] = (1-self.alpha)*self.u_prev[0, 0] + self.alpha*((self.imu_calibration @ self.imu.acceleration)[1])
                elif self.is_turn:
                    self.u[1, 0] = (1-self.alpha)*self.u_prev[1, 0] + self.alpha*self.imu.gyro[2]*8  # NOTE CONSTANT ADDED FOUND IN SCALING
            self.u_prev = self.u.copy()
            # print("u: ", self.u)
            timestamp = time.time()
            # dt = timestamp - self.last_time
            dt = self.interval
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

    def stop(self):
        self.running = False
        self.join()


def control_to_pose(pose_thread, robot, target_x, target_y,
                    kp_lin=5, ki_lin=2,
                    kp_ang=10, ki_ang=2,
                    max_power=30, min_power=10,
                    pos_tolerance=0.06, ang_tolerance=0.2):

    pose_thread.robot_turn()
    integral_error = 0

    while True:
        pose = pose_thread.get_pose()
        x, y, theta = pose[0, 0], pose[1, 0], pose[4, 0]
        desired_theta = np.arctan2(target_y - y, target_x - x)
        error = (desired_theta - theta + np.pi) % (2 * np.pi) - np.pi
        integral_error += error * pose_thread.interval

        if abs(error) < ang_tolerance:
            break

        power = kp_ang * error + ki_ang * integral_error
        power = max(min(power, max_power), -max_power)
        if abs(power) < min_power:
            power = min_power * np.sign(power)
        if desired_theta > 0:
            power = -power
        robot.setMotor(int(power), -int(power))
        time.sleep(pose_thread.interval)

    robot.stop()
    pose_thread.robot_move()
    integral_error = 0

    while True:
        pose = pose_thread.get_pose()
        x, y, theta = pose[0, 0], pose[1, 0], pose[4, 0]
        dx = target_x - x
        dy = target_y - y
        distance = np.sqrt(dx**2 + dy**2)
        error = distance
        integral_error += error * pose_thread.interval

        if distance < pos_tolerance:
            break

        power = kp_lin * error + ki_lin * integral_error
        power = max(min(power, max_power), -max_power)
        if abs(power) < min_power:
            power = min_power * np.sign(power)

        if power < 0:
            power = -power

        robot.setMotor(-int(power), -int(power))
        time.sleep(pose_thread.interval)

    robot.stop()
    pose_thread.robot_still()
