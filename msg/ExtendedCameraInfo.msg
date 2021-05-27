# Contain the standard  sensor_msgs/CameraInfo and extended information about the image acquisition parameters.
# This includes information about the Exposure, Gain, Black level and White balance.

sensor_msgs/CameraInfo camera_info # contains a header and calibration parameters

float64 exposure_time # in microseconds

float64 gain_red # unit is device-specific
float64 gain_green # unit is device-specific
float64 gain_blue # unit is device-specific

float64 black_level_red # unit is device-specific
float64 black_level_green # unit is device-specific
float64 black_level_blue # unit is device-specific

float64 white_balance_red # unit is device-specific
float64 white_balance_green # unit is device-specific
float64 white_balance_blue # unit is device-specific
