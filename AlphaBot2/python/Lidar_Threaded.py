import time
import csv
from rplidar import RPLidar
import threading

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

# --- Main logic ---
def main():
    lidar = RPLidar('/dev/ttyUSB0', baudrate=460800)
    lidar.start_motor()
    time.sleep(1)

    lidar_thread = LidarThread(lidar, interval=1.0)
    lidar_thread.start()

    scan_list = []

    print("Starting scan collection...")
    for i in range(4):
        print(f"Waiting for scan {i+1}...")
        time.sleep(5)  # Wait 5 seconds before each scan
        scan = lidar_thread.get_latest_scan()
        scan_list.append(scan)
        print(f"Scan {i+1} collected: {len(scan)} points")

    print("Stopping Lidar...")
    lidar_thread.stop()
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

    # Save scans to CSV files
    print("Saving scans to CSV...")
    for i, scan in enumerate(scan_list):
        with open(f'scan_{i}.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Confidence', 'Angle', 'Distance'])
            writer.writerows(scan)

    print("Done.")

if __name__ == "__main__":
    main()
