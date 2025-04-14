import time
import csv
from rplidar import RPLidar
import threading
from AlphaBot2 import AlphaBot2
import board
import adafruit_adxl34x


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


def log_acceleration_data(accelerometer, duration=0.7, interval=0.1, filename='accel.csv'):
    """
    Logs acceleration data for a specified duration and interval, and saves to CSV.
    """
    samples = []
    start_time = time.time()
    while time.time() - start_time < duration:
        ax, ay, az = accelerometer.acceleration
        timestamp = time.time() - start_time
        samples.append((timestamp, ax, ay, az))
        time.sleep(interval)

    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time', 'Accel_X', 'Accel_Y', 'Accel_Z'])
        writer.writerows(samples)
    print(f"Acceleration data saved to {filename}")


# --- Main ---
def main():
    # Initialize Lidar
    lidar = RPLidar('/dev/ttyUSB0', baudrate=460800)
    lidar.start_motor()
    time.sleep(1)
    lidar_thread = LidarThread(lidar, interval=1.0)
    lidar_thread.start()

    # Initialize AlphaBot2
    robot = AlphaBot2()
    robot.PA = 30
    robot.PB = 30

    # Initialize Accelerometer
    i2c = board.I2C()
    accelerometer = adafruit_adxl34x.ADXL343(i2c)

    for i in range(4):
        collect_lidar_scan(lidar_thread= lidar_thread,
                           output_filename=f"lidar_move_and_scan_{i}.csv"
        )
        time.sleep(1)
        print("Move Forward")
        robot.forward()
        
        # Log accelerometer data while moving forward
        log_acceleration_data(
            accelerometer,
            duration=0.7,
            interval=0.05,
            filename=f"accel_move_and_scan_{i}.csv"
        )

        robot.stop()
        time.sleep(0.5)
        print("Turn Left")
        robot.left()
        time.sleep(0.15)
        robot.stop()

    lidar_thread.stop()
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

if __name__ == "__main__":
    main()
