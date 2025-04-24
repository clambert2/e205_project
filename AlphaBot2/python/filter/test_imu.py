from imu import IMU
import time
import numpy as np
import matplotlib.pyplot as plt

# Write some imu test code that prints to terminal the values of the imu and integrated values

if __name__=='__main__':
    imu = IMU()
    time.sleep(1)
    print("IMU Test")
    print("Press Ctrl+C to stop the test")
    print("")
    samples = imu.get_data()
    print("")

    position = np.zeros(3)  # [x, y, theta]
    velocity = np.zeros(3)  # [vx, vy, vtheta]
    last_timestamp = 0.00
    time_passed = 0.00
    for i, sample in enumerate(samples):
        timestamp, ax, ay, az, gx, gy, gz = sample
        print(f"Timestamp: {timestamp:.3f} s")
        print(f"Acceleration: ax={ax:.3f}, ay={ay:.3f}, az={az:.3f}")
        print(f"Gyroscope: gx={gx:.3f}, gy={gy:.3f}, gz={gz:.3f}")
        print("")

        dt = (timestamp - timestamp) if  i < 1 else (timestamp - samples[i-1][0])
        time_passed += dt
        print(f"Time delta: {dt:.3f} s")
        velocity[0] += ax * dt
        velocity[1] += ay * dt
        position[0] += velocity[0] * dt
        position[1] += velocity[1] * dt
        position[2] += gz * dt
        print(f"Integrated Position: x={position[0]:.3f}, y={position[1]:.3f}, theta={position[2]:.3f}")
        print(f"Integrated Velocity: vx={velocity[0]:.3f}, vy={velocity[1]:.3f}, vtheta={position[2]:.3f}")
        print(f"Time passed: {time_passed:.3f} s")

