#!/usr/bin/env python
# coding: utf-8
import rospy
import roslib

from kitchen.msg import obj
from kitchen.msg import objs
from sensor_msgs import msg
from sensor_msgs.msg import RegionOfInterest

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
from itertools import chain

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
os.environ['CUDA_VISIBLE_DEVICES'] = "0,1"
from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

from keras.backend.tensorflow_backend import set_session
import tensorflow as tf
config = tf.ConfigProto()
config.gpu_options.allow_growth = True  # dynamically grow the memory used on the GPU
config.log_device_placement = True  # to log device placement (on which device the operation ran)
sess = tf.Session(config=config)
set_session(sess)  # set this TensorFlow session as the default session for Keras

import sys
import random
import math
import skimage.io
import time
import mrcnn.config
from mrcnn import utils
#import model as modellib
import matplotlib.pyplot as plt
ROOT_DIR = os.path.abspath("./")

sys.path.append(ROOT_DIR)
from mrcnn import utils
import mrcnn.model as modellib
#import coco
from mrcnn.config import Config
from mrcnn import visualize
from PIL import Image


def random_colors(N):
    np.random.seed(1)
    colors=[tuple(255*np.random.rand(3)) for _ in range(N)]
    return colors

def apply_mask(image, mask, color,label):
    """Apply the given mask to the image.
    """
    #if(mask[:][0]==True):
    #print("img::",mask[:][240])
    alpha=0.2
    for n, c in enumerate(color):
        image[:, :, n] = np.where(
            mask == 1,
            image[:, :, n] *(1 - alpha) + alpha * c,
            image[:, :, n]
        )
    
    return image
##class_names = ['BG','plate', 'pan','vegetablebowl','broccoli','souppothandle','panhandle','beef','nethandle','seasoningbottle','seasoningbowl']
detect_obj = obj()
detect_obj.id = []
detect_obj.classname = []
detect_obj.score = []
detect_obj.roi = []

detect_objs = objs()


#detect_objs.header.nsecs = now.nsecs

def display_instances(image,boxes,masks,ids,names,scores):
    n_instances=boxes.shape[0]
    if not n_instances:
        print('No instances to display')
    else:
        assert boxes.shape[0] == masks.shape[-1] == ids.shape[0]

    detect_obj.id = []
    detect_obj.classname = []
    detect_obj.score = []
    detect_obj.roi = []
    detect_obj.masks = []
    detect_objs.objects_vector = []
    #detect_objs.header.stamp.secs = 0
    #detect_objs.header.stamp.nsecs = 0
    

    colors=random_colors(n_instances)
    height, width = image.shape[:2]
    for i,color in enumerate(colors):
        #if scores[i]>0.85:
        if not np.any(boxes[i]):
            continue
        y1,x1,y2,x2=boxes[i]
        box = RegionOfInterest()
        box.x_offset = np.asscalar(x1)
        box.y_offset = np.asscalar(y1)
        box.height = np.asscalar(y2 - y1)
        box.width = np.asscalar(x2 - x1)
        detect_obj.roi.append(box)

        masklist = msg.Image()
        cimgmask =np.uint8(masks[:, :, i])
        temp = cimgmask*255
        bridge1=CvBridge()
        masklist = bridge1.cv2_to_imgmsg(temp, encoding="passthrough")
        detect_obj.masks.append(masklist)
        

        mask=masks[:,:,i]
        label=names[ids[i]]
        image=apply_mask(image,mask,color,label)

        detect_obj.id.append(ids[i])
        detect_obj.classname.append(label)
        detect_obj.score.append(scores[i])

        image=cv2.rectangle(image,(x1,y1),(x2,y2),color,2)
        print("***********",[x1,y1,x2,y2])
        score=scores[i] if scores is not None else None
        caption='{}{:.2f}'.format(label,score) if score else label
        image=cv2.putText(image,caption,(x1,y1),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)

    detect_objs.objects_vector.append(detect_obj)
    
    objs_msg = objs()

    return image

sys.path.append(os.path.join(ROOT_DIR, "samples/coco/"))  # To find local version
MODEL_DIR = os.path.join(ROOT_DIR, "logs")
COCO_MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco_0160.h5")
if not os.path.exists(COCO_MODEL_PATH):
    print('cannot find coco_model')

class CocoConfig(Config):
    # 命名配置
    NAME = "coco"
    # 输入图像resing
    IMAGE_MIN_DIM = 540
    IMAGE_MAX_DIM = 960

    # 使用的GPU数量。 对于CPU训练，请使用1
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

    batch_size = GPU_COUNT * IMAGES_PER_GPU
    # STEPS_PER_EPOCH = int(train_images / batch_size * (3 / 4))
    STEPS_PER_EPOCH=1000
    VALIDATION_STEPS = STEPS_PER_EPOCH // (1000 // 50)
    NUM_CLASSES = 1 + 8  # 必须包含一个背景（背景作为一个类别） Pascal VOC 2007有20个类，前面加1 表示加上背景

    scale = 1024 // IMAGE_MAX_DIM
    RPN_ANCHOR_SCALES = (32 // scale, 64 // scale, 128 // scale, 256 // scale, 512 // scale)
    RPN_NMS_THRESHOLD = 0.6  # 0.6
    RPN_TRAIN_ANCHORS_PER_IMAGE = 256 // scale
    MINI_MASK_SHAPE = (56 // scale, 56 // scale)

    TRAIN_ROIS_PER_IMAGE = 200 // scale

    DETECTION_MAX_INSTANCES = 100 * scale * 2 // 3
    DETECTION_MIN_CONFIDENCE = 0.6


#class InferenceConfig(coco.CocoConfig):
class InferenceConfig(CocoConfig):  ###zhu
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

config = InferenceConfig()
config.display()

#model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)
model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)
# Load weights trained on MS-COCO
model.load_weights(COCO_MODEL_PATH, by_name=True)

#class_names = ['BG','pan','beef','plate', 'vegetablebowl','broccoli','souppothandle','panhandle','nethandle','seasoningbottle','seasoningbowl']
class_names = ['BG','plate', 'broccoli', 'net', 'pot',  'bowl', 'pan','beef', 'bottle']
def model_detection(img,img_hd,depth_img):
    
    bridge=CvBridge()
    detect_objs.rgb_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    detect_objs.depth_img = bridge.cv2_to_imgmsg(depth_img,"passthrough")
    detect_objs.rgb_img_hd =  bridge.cv2_to_imgmsg(img_hd, encoding="bgr8")
    now = rospy.Time.now()
    detect_objs.header.stamp = now
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!",detect_objs.header)
    print("****************************",now)
    detect_objs.header.frame_id = "map"
    
    results=model.detect([img],verbose=0)
    r=results[0]
    frame=display_instances(
            img,r['rois'], r['masks'], r['class_ids'], 
                        class_names, r['scores']
    )

    #print("----------------22---------------",detect_objs)
    detect_objs_pub = rospy.Publisher("processor/objs",objs,queue_size=1)
    detect_objs_pub.publish(detect_objs)
    cv2.imshow("img",frame)
    cv2.waitKey(1)

