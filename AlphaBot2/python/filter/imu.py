import threading
import time
import csv
import board
import numpy as np
from queue import Queue
from scipy.signal import butter, filtfilt
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3
from adafruit_bus_device.i2c_device import I2CDevice


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
                a = self.imu.acceleration[1]
                w = self.imu.gyro[2] * 8 # NOTE CONSTANT ADDED FOUND IN SCALING
                data_point = (timestamp, a, w)
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
    def __init__(self, interval=0.01):
        self.i2c = board.I2C()
        self.imu = LSM6DS3(self.i2c)
        self.last_timestamp = 0.00
        self.time_passed = 0.00

        # Manually set accel and gyro range via I2C
        with I2CDevice(self.i2c, 0x6A) as device:
            # ACCEL: ODR = 104Hz (0b0100), FS = ±2g (0b00) => 0b01000000 = 0x40
            device.write(bytes([0x10, 0x40]))

            # GYRO: ODR = 104Hz (0b0100), FS = ±1000 dps (0b11) => 0b01001100 = 0x4C
            device.write(bytes([0x11, 0x4C]))

        self.thread = IMUThread(self.imu, interval=interval)
        self.thread.start()

    def get_data(self):
        return self.thread.get_data()

    def stop(self):
        self.thread.stop()
    
    def ewma_filter(self, data, alpha=0.75):
        for i in range(1, len(data)):
            data[i] = alpha * data[i-1] + (1 - alpha) * data[i]
        return data

    def clear_data(self):
        self.thread.queue.queue.clear()
