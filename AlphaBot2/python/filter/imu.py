import threading
import time
import csv
import board
import numpy as np
from queue import Queue
from scipy.signal import butter, filtfilt
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3


class IMUThread(threading.Thread):
    def __init__(self, imu, interval=0.05, max_queue_size=100):
        super().__init__()
        self.imu = imu
        self.interval = interval
        self.running = True
        self.lock = threading.Lock()
        self.queue = Queue(maxsize=max_queue_size)

    def run(self):
        while self.running:
            try:
                timestamp = time.time()
                ax, ay, az = self.imu.acceleration
                gx, gy, gz = self.imu.gyro
                data_point = (timestamp, ax, ay, az, gx, gy, gz)
                if not self.queue.full():
                    self.queue.put(data_point)
            except Exception as e:
                print("IMU error:", e)
            time.sleep(self.interval)

    def get_data(self):
        data = []
        while not self.queue.empty():
            data.append(self.queue.get())
        return data

    def stop(self):
        self.running = False
        self.join()


class IMU:
    def __init__(self, interval=0.05):
        self.i2c = board.I2C()
        self.imu = LSM6DS3(self.i2c)
        self.imu.set_range(LSM6DS3.ACCEL_RANGE_2_G, LSM6DS3.GYRO_RANGE_1000_DPS)
        self.thread = IMUThread(self.imu, interval=interval)
        self.thread.start()

    def get_data(self):
        return self.thread.get_data()

    def stop(self):
        self.thread.stop()

    def filter_data(self, data, cutoff, order):
        nyquist = 0.5 * 100
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        filtered_data = {
            'Time': data[0],
            'Accel_X': filtfilt(b, a, data[1]),
            'Accel_Y': filtfilt(b, a, data[2]),
            'Accel_Z': filtfilt(b, a, data[3]),
            'Gyro_X': filtfilt(b, a, data[4]),
            'Gyro_Y': filtfilt(b, a, data[5]),
            'Gyro_Z': filtfilt(b, a, data[6]),
        }
        return filtered_data
