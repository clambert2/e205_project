import time
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
    lidar.start_motor()

    # Create lidar scan list
    scan_list = []

    for i in range(4):
        # Get Lidar scan data
        lidar.append(lidar.iter_scans())
        Ab.forward()
        time.sleep(1)
        Ab.stop()
        time.sleep(1)
        Ab.right()
        time.sleep(1)
        Ab.stop()
        time.sleep(1)
