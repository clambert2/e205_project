from lidar import Lidar
from imu import IMU
import utils
from map import Map
from pose_estimator import PoseEstimator


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

'''
while True:
    # 1. Lidar scan
    lidar_data = robot.lidar.scan()
    
    # 2. Measurement Update for EKF
    robot.ekf.update(lidar_data)
    
    # 3. Update Map
    robot.map.update(lidar_data, robot.position)
    
    # 4. Path Planning for next movement
    next_position = robot.map.plan_path(robot.position)
    
    # 5. Move robot to next position
    robot.move_to(next_position)
    
    # 6. While robot is moving update prediction for EKF
    while not robot.reached_destination(next_position):
        robot.ekf.predict()
'''

if __name__=='__main__':
    # Initialize the robot components
    imu = IMU()
    
    # Main loop
    while True:
        # 1. Lidar scan
        lidar_data = lidar.scan()
        
        # 2. Measurement Update for EKF
        ekf.update(lidar_data)
        
        # 3. Update Map
        map.update(lidar_data, robot.position)
        
        # 4. Path Planning for next movement
        next_position = map.plan_path(robot.position)
        
        # 5. Move robot to next position
        robot.move_to(next_position)
        
        # 6. While robot is moving update prediction for EKF
        while not robot.reached_destination(next_position):
            ekf.predict()