from rplidar import RPLidar

PORT_NAME = "/dev/ttyUSB0"  # Default for Raspberry Pi

def run_lidar(print_data=False):
    """Reads and prints data from RP Lidar."""
    # Initialize the Lidar
    lidar = RPLidar(PORT_NAME, baudrate=460800)
    print("Lidar connected! Reading data... (Press Ctrl+C to stop)")

    # Start the Lidar and connect
    lidar.stop()
    lidar.connect()

    # Get the health and info of the Lidar
    print(lidar.get_health())
    print(lidar.get_info())
    # Start the motor and collect a scaning
    lidar.start_motor()
    scan = lidar.iter_scans()

    # Print the scan data
    if print_data:
        for s in scan:
            print(s)
            break

    # Stop the Lidar and disconnect
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

    # Return the scan data
    return scan

if __name__ == "__main__":
    run_lidar()
