from rplidar import RPLidar

# Set your Lidar's USB port (Check with `ls /dev/ttyUSB*`)
PORT_NAME = "/dev/ttyUSB0"  # Default for Raspberry Pi

def run_lidar():
    """Reads and prints data from RP Lidar."""

    lidar = RPLidar(PORT_NAME, baudrate=460800)
    print("✅ Lidar connected! Reading data... (Press Ctrl+C to stop)")
    lidar.stop()
    lidar.connect()
    print(lidar.get_health())
    print(lidar.get_info())
    lidar.start_motor()
    scan = lidar.iter_scans()
    for s in scan:
        print(s)
        break


    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

if __name__ == "__main__":
    run_lidar()
