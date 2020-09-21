# Copyright (c) 2017 Rooin Corporation
std_msgs/Header header    # timestamp in header is the time the sensor captured the raw data
obj[] objects_vector   # Object array
float32 inference_time_ms # inference time of this frame. the unit is millisecond.
sensor_msgs/Image rgb_img # Instance masks as Image
sensor_msgs/Image rgb_img_hd # Instance masks as Image
sensor_msgs/Image depth_img # Instance masks as Image
