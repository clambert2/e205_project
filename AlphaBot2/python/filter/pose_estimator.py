import numpy as np
import time
import csv
from imu import IMU

class PoseEstimator:

    def __init__(self):
        # Initialize EKF parameters all units are in millimeters
        self.x = np.zeros((3, 1))
        self.x_bar = np.zeros((3, 1))

        self.S = np.eye(3)
        self.S_bar = np.eye(3)

        self.Gx = np.eye(3)
        self.Gu = np.array([[np.cos(self.x[2,0]), 0, 0],
                            [0, np.sin(self.x[2,0]), 0],
                            [0, 0, 1]
                            ])

        self.R = np.array([[10, 0, 0],
                           [0, 10, 0],
                           [0, 0, 10]
                           ])
        
        self.H = np.eye(3)
        self.Q = np.array([[20, 0, 0],
                           [0, 20, 0],
                           [0, 0, 20]
                           ])
        self.K = 1

    def prediction(self, u, dt):
        self.Gu[3, 2] = dt
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
