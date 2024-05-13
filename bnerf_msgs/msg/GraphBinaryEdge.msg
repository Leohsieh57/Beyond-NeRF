time source_stamp # transformation from source to target
time target_stamp

geometry_msgs/Transform mean # mean measurement of transformation
float64[36] covariance

uint8 type
uint8 LIDAR_TO_LIDAR=0
uint8 LIDAR_TO_CAMERA=1
uint8 IMU=2
