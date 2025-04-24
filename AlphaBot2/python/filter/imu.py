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
        self.pose = [0, 0, 0]  # [x, y, theta]
        self.vel = [0, 0, 0]
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

    def get_pose(self):
        return self.pose
    
    def get_velocity(self):
        return self.velocity
    
    def update_pose(self):
        # position = np.zeros(3)  # [x, y, theta]
        # velocity = np.zeros(3)  # [vx, vy, vtheta]
        samples = self.get_data()
        for i, sample in enumerate(samples):
            print(self.pose)
            timestamp, ax, ay, az, gx, gy, gz = sample
            dt = (timestamp - timestamp) if  i < 1 else (timestamp - samples[i-1][0])
            self.vel[0] += ax * dt
            self.vel[1] += ay * dt
            self.vel[2] = gz
            self.pose[0] += self.vel[0] * dt
            self.pose[1] += self.vel[1] * dt
            self.pose[2] += gz * dt * 8
        return self.pose

    def clear_data(self):
        self.thread.queue.queue.clear()
        self.velocity = [0, 0, 0]
