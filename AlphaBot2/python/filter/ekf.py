import numpy as np
import time
import csv
from imu import IMU

class EKF:

    def __init__(self):
        # Initialize EKF parameters all units are in meters and radians
        self.x = np.zeros((3, 1))
        self.x_bar = np.zeros((3, 1))

        self.S = np.eye(3)
        self.S_bar = np.eye(3)

        self.Gx = np.eye(3)
        self.Gu = np.array([[np.cos(self.x[2,0]), 0],
                            [np.sin(self.x[2,0]), 0],
                            [0, 1]
                            ])

        self.R = np.eye(2)*0.2
        self.H = np.eye(3)
        self.Q = np.eye(3)*0.1
        self.K = 1

    def prediction(self, imu_measurement, is_turn = False, is_move = False):
        for i, sample in enumerate(imu_measurement):
            timestamp, a, w = sample
            if is_turn:
                a = 0
            if is_move:
                w = 0
            dt = (timestamp - timestamp) if  i < 1 else (timestamp - imu_measurement[i-1][0])
            print(f"dt: {dt}")
            u = np.array([[a], [w]])
            print(f'u: {u}')
            self.Gu[0, 0] = dt * np.cos(self.x[2, 0])
            self.Gu[1, 0] = dt * np.sin(self.x[2, 0])
            self.Gu[2, 1] = dt
            self.x_bar = self.Gx @ self.x_bar + self.Gu @ u
            self.S_bar = self.Gx @ self.S_bar @ self.Gx.T + self.Gu @ self.R @ self.Gu.T
        return self.x_bar[0, 0], self.x_bar[1, 0], self.x_bar[2, 0]

    def measurement_update(self, z):
        self.K = self.S_bar @ self.H.T @ np.linalg.inv(self.H @ self.S_bar @ self.H.T + self.Q)
        self.x = self.x_bar + self.K @ (z - self.H @ self.x_bar)
        self.S = (np.eye(3) - self.K @ self.H) @ self.S_bar
        self.S_bar = self.S
        self.x_bar = self.x
        return self.x[0, 0], self.x[1, 0], self.x[2, 0]
