#!/usr/bin/env python
from kitchen.msg import objs
from kitchen.msg import obj

class Tracker:
    def __init__(self):
        self.trackSucess = False

    def track(self, vis, objs):
        self.trackSucess = True
        trackedObjs = objs
        return trackedObjs
