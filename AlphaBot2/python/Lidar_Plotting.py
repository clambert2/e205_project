import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar

PORT = '/dev/ttyUSB0'

lidar = RPLidar(PORT)

def get_scan_data():
    return lidar.iter_scans()

print(get_scan_data())
