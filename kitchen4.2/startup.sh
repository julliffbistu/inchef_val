#!/bin/bash

gnome-terminal -- bash -c "roscore; exec bash;"
sleep 3

gnome-terminal -- bash -c "roslaunch kinect2_bridge kinect2_bridge.launch ; exec bash;"
sleep 1

gnome-terminal -- bash -c "cd src/coco-Mask_RCNN/; python samples/ros_mask.py; exec bash;"
sleep 1

gnome-terminal -- bash -c "cd src/coco-Mask_RCNN/; python samples/DL_percessing.py; exec bash;"
sleep 1

gnome-terminal -- bash -c "cd src/script; python inchef_calibration.py; exec bash;"
sleep 1

gnome-terminal -- bash -c "source ../../devel/setup.bash; cd src/sanity; python demo_sanity.py; exec bash;"
sleep 1

gnome-terminal -- bash -c "cd src; python luban_bridge.py; exec bash;"
sleep 1

gnome-terminal -- bash -c "cd src; python topicMonitor.py; exec bash;"
sleep 1

gnome-terminal -- bash -c "cd src; python maintenanceNode.py; exec bash;"
sleep 1

gnome-terminal -- bash -c "cd src/ag_perception/src; python ag_perception.py; exec bash;"
sleep 1

gnome-terminal -- bash -c "source ../../devel/setup.bash; rosrun log_service log_service; exec bash;"
