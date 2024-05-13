time start_stamp
time end_stamp

geometry_msgs/Transform mean
float64[36] covariance

uint8 type
uint8 INVALID=0
uint8 LIDAR_TO_LIDAR=1
uint8 CAM_TO_LIDAR=2
uint8 IMU=3