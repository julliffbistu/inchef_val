#hehe@29082020 
1) delete config.yaml
2) in inchef1.0, copy this file "config_inchef01.yaml" and rename to "config.yaml"
3) in inchef3.0, copy this file "config_inchef03.yaml" and rename to "config.yaml"
4) if use YOLO model, under root "~/catkin_ws" directory, use command "python src/kitchen3.0/src/sanity/demo.py"
   else if use maskRCNN model, 
	1) under "~/catkin_ws/src/kitchen3.0/src/coco-Mask_RCNN" directory, use command "python samples/ros_mask.py"
	2) under root "~/catkin_ws" directory, use command "python src/kitchen3.0/src/sanity/demo_sanity.py"
GOOD LUCK!
