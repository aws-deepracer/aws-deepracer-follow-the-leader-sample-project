# Custom message to combine different sensor messages.
#
# This message holds a list of sensor_msgs/Image objects that are
# independently collected from different camera sensors. And an
# array of distance values read from the LiDAR sensor.
sensor_msgs/Image[] images    # Image data read from cameras.
float32[] lidar_data          # LiDAR distance data read from LiDAR sensor.