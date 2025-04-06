import time
import csv
from AlphaBot2 import AlphaBot2
from rplidar import RPLidar
from Lidar_Test import run_lidar

if __name__ == "__main__":

    # Create AlphaBot2 object
    Ab = AlphaBot2()

    # Set the robot to move a bit slower
    Ab.PA = 20
    Ab.PB = 20

    # Create Lidar object
    lidar = RPLidar("/dev/ttyUSB0", baudrate=460800)
    lidar.stop()
    lidar.connect()
    print(lidar.get_info())
    print(lidar.get_health())
    time.sleep(1)

    # Create lidar scan list and initialize scan
    scan_list = []

    # Movement parameters
    forward_time = 0.5  # seconds
    turn_time = 0.2  # seconds

    # Move the robot and collect data
    for i in range(4):
        # Get Lidar scan data
        time.sleep(2)
        lidar.stop()
        lidar.start_motor()
        scan = lidar.iter_scans()
        scan_list.append(next(scan))
        Ab.forward()
        time.sleep(forward_time)
        Ab.stop()
        time.sleep(2)
        lidar.stop()
        lidar.start_motor()
        scan = lidar.iter_scans()
        scan_list.append(next(scan))
        time.sleep(1)
        Ab.forward()
        time.sleep(forward_time)
        Ab.stop()
        time.sleep(2)
        lidar.stop()
        lidar.start_motor()
        scan = lidar.iter_scans()
        scan_list.append(next(scan))
        time.sleep(1)
        Ab.right()
        time.sleep(turn_time)
        Ab.stop()

    # Stop the Lidar and disconnect
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

    # Save scan data to CSV file (new file for each scan)
    for i, scan in enumerate(scan_list):
        with open(f'scan_{i}.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Confidence','Angle', 'Distance'])
            writer.writerows(scan)
