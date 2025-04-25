import numpy as np
import time
import csv
from imu import IMU

class PoseEstimator:

    def __init__(self):
        self.Gx = np.array()
        self.Gu = np.array()
        self.H = np.array()
        self.R = np.array()
        self.Q = np.array()
        self.S = np.array()
        self.S_bar = np.array()
        self.K = np.array()
        self.x = np.zeros((3, 1))
        self.x_bar = np.zeros((3, 1))


    def prediction

    def measurement_update

