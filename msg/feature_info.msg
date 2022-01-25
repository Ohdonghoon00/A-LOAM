Header header

# Line
float64[] p1_x
float64[] p1_y
float64[] p1_z

float64[] p2_x
float64[] p2_y
float64[] p2_z

# Plane
float64[] normal_x
float64[] normal_y
float64[] normal_z

float64[] centroid_x
float64[] centroid_y
float64[] centroid_z

float64[] scale

# imu preintegration
float32 gyro_rx
float32 gyro_ry
float32 gyro_rz
float32 gyro_x
float32 gyro_y
float32 gyro_z

# cloud msg
sensor_msgs/PointCloud2 cloud_undistortion
