terminal 0
roscore
terminal 1
roslaunch kinect2_bridge kinect2_bridge.launch sensor:=019596552647


terminal 2
cd ~/catkin_ws/src/kitchen3.0/src/coco-Mask_RCNN/
python samples/ros_mask.py 

terminal 3
cd ~/catkin_ws/src/kitchen3.0/src/coco-Mask_RCNN/
python samples/DL_percessing.py


terminal 4
cd ~/catkin_ws/src/kitchen3.0/src/script
python inchef_calibration.py

terminal 5
cd ~/catkin_ws/src/kitchen3.0/src/
python luban_bridge.py 


terminal 6
cd ~/catkin_ws/src/kitchen3.0/src/ag_perception/src
python ag_perception.py
