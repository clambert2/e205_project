# import numpy as np
# import matplotlib.pyplot as plt
# from rplidar import RPLidar

# PORT = '/dev/ttyUSB0'

# lidar = RPLidar(PORT)

# def get_scan_data():
#     for i, scan in enumerate(lidar.iter_scans()):
#         print(f"Scan {i+1}: {len(scan)} points")
#     return scan

# print(get_scan_data())

from rplidar import RPLidar

# Set your Lidar's USB port (Check with `ls /dev/ttyUSB*`)
PORT_NAME = "/dev/ttyUSB0"  # Default for Raspberry Pi

def run_lidar():
    """Reads and prints data from RP Lidar."""

    lidar = RPLidar(PORT_NAME, baudrate=460800)
    print("✅ Lidar connected! Reading data... (Press Ctrl+C to stop)")

    lidar.connect()
    print(lidar.get_health())
    print(lidar.get_info())
    lidar.start_motor()

    for scan in lidar.iter_scans():
        print("\n🔄 New Scan:")
        for (_, angle, distance) in scan:
            print(f"Angle: {angle:.2f}°, Distance: {distance:.2f} mm")

    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

if __name__ == "__main__":
    run_lidar()
