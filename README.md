# State Estimation and Mapping for Mobile Robot

## Authors
- Charlie Lambert 
- Jasper Cox 
- Anthony Tran 

## Project Description

This project implements a modular state estimation and mapping system for a mobile robot using a Raspberry Pi-based platform. The robot integrates odometry, IMU data, and LiDAR to build a global map of its environment while tracking its own pose. Core functionality includes real-time data fusion with an Extended Kalman Filter (EKF), scan matching via the Open3D library’s ICP algorithm, and autonomous exploration using frontier detection and path planning. The system is designed for periodic corrections, where the robot stops to collect high-quality LiDAR scans that are aligned to a global map for improved localization.

## Repository Structure

All currently relevant code is located in the `main` branch.

- **`filter/`**: Contains the core filtering and motion control logic.
  - Use `main.py` in this directory to operate the robot. This script handles movement and filtering.
- **`tests/`**: Includes scripts for individual operations:
  - Taking a LiDAR scan and saving it as a CSV
  - Matching scans using the Open3D ICP library
  - Building a global map
  - Detecting frontiers
  - Generating a path to the next exploration point
