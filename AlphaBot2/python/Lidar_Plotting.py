import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar

PORT = '/dev/ttyUSB0'

lidar = RPLidar(PORT)

def get_scan_data():
    for i, scan in enumerate(lidar.iter_scans()):
        print(f"Scan {i+1}: {len(scan)} points")
    return scan

print(get_scan_data())
