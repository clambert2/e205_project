import time
import csv
from rplidar import RPLidar
import threading
from AlphaBot2 import AlphaBot2
import board
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3
import numpy as np


# --- Thread class for collecting Lidar scans ---
class LidarThread(threading.Thread):
    def __init__(self, lidar, interval=1.0):
        super().__init__()
        self.lidar = lidar
        self.interval = interval
        self.running = True
        self.lock = threading.Lock()
        self.latest_scan = []
        self.measure_iter = lidar.iter_measures()

    def run(self):
        while self.running:
            scan = []
            start_time = time.time()
            while time.time() - start_time < self.interval:
                try:
                    new_scan, quality, angle, distance = next(self.measure_iter)
                    if quality > 0:
                        scan.append((quality, angle, distance))
                except Exception as e:
                    print("Lidar error:", e)
                    break

            with self.lock:
                self.latest_scan = scan

    def get_latest_scan(self):
        with self.lock:
            return list(self.latest_scan)

    def stop(self):
        self.running = False
        time.sleep(0.1)

def collect_lidar_scan(lidar_thread, timeout=10.0, output_filename='scan.csv'):
    '''
    Collects a full 360° Lidar scan and saves it to a CSV file.

    Args:
        lidar_thread (LidarThread): The thread collecting Lidar data.
        timeout (float): Max time to wait for a full scan.
        output_filename (str): File to save the scan.
    '''
    print("Waiting for full 360° scan...")

    start_time = time.time()
    all_points = []
    angle_set = set()
    
    while time.time() - start_time < timeout:
        scan = lidar_thread.get_latest_scan()

        # Filter and append new angles
        for point in scan:
            quality, angle, distance = point
            rounded_angle = int(angle)  # round to nearest degree
            if rounded_angle not in angle_set:
                angle_set.add(rounded_angle)
                all_points.append(point)

        if len(angle_set) >= 360:
            print(f"Full scan collected with {len(all_points)} points.")
            break

        time.sleep(0.05)

    if len(angle_set) < 360:
        print(f"Warning: Incomplete scan. Only got {len(angle_set)} unique angles.")

    # Save to CSV
    with open(output_filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Confidence', 'Angle', 'Distance'])
        writer.writerows(all_points)

    print(f"Scan saved to {output_filename}")


def log_imu_data(imu, duration=0.7, interval=0.1, accel = True, filename='accel.csv'):
    samples = []
    start_time = time.time()
    while time.time() - start_time < duration:
        if accel:
            x, y, z = imu.acceleration
        else:
            x, y, z = imu.gyro
        timestamp = time.time() - start_time
        samples.append((timestamp, x, y, z))
        time.sleep(interval)

    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        if accel:
            writer.writerow(['Time', 'Accel_X', 'Accel_Y', 'Accel_Z'])
        else:
            writer.writerow(['Time', 'Gyro_X', 'Gyro_Y', 'Gyro_Z'])
        writer.writerows(samples)
    print(f"Acceleration data saved to {filename}")
    return samples


# --- Main ---
def main():
    # # Initialize Lidar
    # lidar = RPLidar('/dev/ttyUSB0', baudrate=460800)
    # lidar.start_motor()
    # time.sleep(1)
    # lidar_thread = LidarThread(lidar, interval=1.0)
    # lidar_thread.start()

    # Initialize AlphaBot2
    robot = AlphaBot2()
    robot.PA = 30
    robot.PB = 30

    # Initialize Accelerometer
    i2c = board.I2C()
    imu = LSM6DS3(i2c)

    for i in range(1):
        # collect_lidar_scan(lidar_thread= lidar_thread,
        #                    output_filename=f"lidar_move_and_scan_{i}.csv"
        # )
        # time.sleep(1)

        # Hold robot still for 10 seconds and collect imu data and then zero outputs
        # print("Hold still")
        # robot.stop()
        # samples = log_imu_data(
        #     imu = imu,
        #     duration=10.0,
        #     interval=0.1,
        #     accel=True,
        #     filename=f"accel_move_and_scan_0.csv"
        # )
        # accel_offsets = np.mean(np.array(samples)[:, 1:], axis=0)
        # samples = log_imu_data(
        #     imu = imu,
        #     duration=0.5,
        #     interval=0.1,
        #     accel = False,
        #     filename=f"accel_move_and_scan_0.csv"
        # )
        # gyro_offsets = np.mean(np.array(samples)[:, 1:], axis=0)

        print("Move Forward")
        robot.forward()
        


        # Log accelerometer data while moving forward
        samples = log_imu_data(
            imu = imu,
            duration=0.7,
            interval=0.001,
            accel=True,
            filename=f"accel_move_and_scan_0.csv"
        )
        samples.insert(0, (0, 0, 0, 0))
        velocity = np.array([0.0, 0.0, 0.0])
        position = np.array([0.0, 0.0, 0.0])
        for i in range(len(samples) - 1):
            dt = samples[i+1][0] - samples[i][0]
            accel = np.array(samples[i][1:])
            velocity += dt * accel
            position += dt * velocity

        print(f"Final velocity: {velocity}")
        print(f"Final position: {position}")

        robot.stop()
        time.sleep(0.5)
        print("Turn Left")
        robot.left()
        
        # Log Gyroscope data while turning left
        samples = log_imu_data(
            imu = imu,
            duration=0.15,
            interval=0.001,
            accel=False,
            filename=f"gyro_move_and_scan_0.csv"
        )

        for i in range(len(samples) - 1):
            dt = samples[i+1][0] - samples[i][0]
            gyro = np.array(samples[i][1:])
            position += dt * gyro
        print(f"Final position: {position}")
        print(f"Final position: {np.rad2deg(position)}")

        robot.stop()

    # lidar_thread.stop()
    # lidar.stop()
    # lidar.stop_motor()
    # lidar.disconnect()

if __name__ == "__main__":
    main()
