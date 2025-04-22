from lidar import Lidar
from imu import IMU
import utils
from map import Map


'''
In the main loop there will be an fsm that will cycle between:
1. Lidar scan
2. Measurement Update for EKF
3. Update Map
4. Path Plann for next movement
4. Move robot to next position
    4.1 While robot is moving update prediction for EKF
5. Repeat
'''
