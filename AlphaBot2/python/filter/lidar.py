import threading
import time
import csv
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from rplidar import RPLidar


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
                # try:
                new_scan, quality, angle, distance = next(self.measure_iter)
                if quality > 0:
                    scan.append((quality, angle, distance))
                # except Exception as e:
                #     print("Lidar error:", e)
                #     break

            with self.lock:
                self.latest_scan = scan

    def get_latest_scan(self):
        with self.lock:
            return list(self.latest_scan)

    def stop(self):
        self.running = False
        time.sleep(0.1)
        

class Lidar:
    def __init__(self, port='/dev/ttyUSB0', baudrate=460800, interval=1.0):
        self.lidar = RPLidar(port, baudrate)
        self.print_health()
        self.print_info()
        self.lidar.start_motor()
        time.sleep(1)
        self.thread = LidarThread(self.lidar, interval=interval)
        self.thread.start()

    def get_scan(self, timeout=10.0, save_csv=False, output_filename='scan.csv'):
        start_time = time.time()
        all_points = []
        angle_set = set()

        while time.time() - start_time < timeout:
            scan = self.thread.get_latest_scan()

            # Filter and append new angles
            for point in scan:
                quality, angle, distance = point
                rounded_angle = int(angle)
                if rounded_angle not in angle_set:
                    angle_set.add(rounded_angle)
                    all_points.append(point)

            if len(angle_set) >= 360:
                print(f"Full scan collected with {len(all_points)} points.")
                break

            time.sleep(0.05)

        if len(angle_set) < 360:
            print(f"Warning: Incomplete scan. Only got {len(angle_set)} unique angles.")

        if save_csv:
            # Save to CSV
            with open(output_filename, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Confidence', 'Angle', 'Distance'])
                writer.writerows(all_points)

            print(f"Scan saved to {output_filename}")

        return self.convert_to_cartesian(all_points)

    def convert_to_cartesian(self, scan_points):
        r = np.array([d for _, _, d in scan_points])
        theta = np.radians([a for _, a, _ in scan_points])
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return np.vstack((x, y)).T

    def stop(self):
        self.thread.stop()
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def plot_scans(self, scans):
        plt.figure(figsize=(8, 8))
        for i, scan in enumerate(scans):
            plt.scatter(scan[:, 0], scan[:, 1], label=f'Scan {i+1}', s=1)
        plt.title("Lidar Scans")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.show()

    def match_scans(self, source_scan, target_scan, threshold=0.1, guess=np.eye(4)):
        source_pcd = o3d.geometry.PointCloud()
        target_pcd = o3d.geometry.PointCloud()

        source_pcd.points = o3d.utility.Vector3dVector(
        np.c_[source_scan, np.zeros(len(source_scan))])
        target_pcd.points = o3d.utility.Vector3dVector(
        np.c_[target_scan, np.zeros(len(target_scan))])

        reg_p2p = o3d.pipelines.registration.registration_icp(
            source_pcd, target_pcd, threshold, guess,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        return reg_p2p.transformation

    def transform_scan(self, scan, transformation):
        homogeneous_scan = np.hstack((scan,
                                      np.zeros((scan.shape[0], 1)),
                                      np.ones((scan.shape[0], 1))
                                    ))
        transformed = (transformation @ homogeneous_scan.T).T
        return transformed[:, :2]

    def print_info(self):
        print(self.lidar.get_info())

    def print_health(self):
        print(self.lidar.get_health())
