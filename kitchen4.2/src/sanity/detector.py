#!/usr/bin/env python
from kitchen.msg import objs
from kitchen.msg import obj
from yolo.yolo import YOLODetector

import enum
from enum import IntEnum

@enum.unique
class DetectorType(IntEnum):
    MODEL_YOLOV4 = 1
    MODEL_MASKRCNN = 2

class Detector:
    def __init__(self,detector_type):
        self.dtype = detector_type
        print("add detector type:", self.dtype)
        if(detector_type == DetectorType.MODEL_YOLOV4):
            self.dt = YOLODetector()
        else:
            return NotImplementedError

    def detect(self, vis):
        if(self.dt is None):
            return NotImplementedError
        return self.dt.detect(vis)
  
