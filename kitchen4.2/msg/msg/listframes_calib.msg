std_msgs/Header header    # timestamp in header is the time the sensor captured the raw data
sglframe_calib[] frames_info   # Object array
float32[] rail_mask
sensor_msgs/Image rgb_img_to_pos # Instance masks as Image
sensor_msgs/Image depth_img_to_pos # Instance masks as Image

