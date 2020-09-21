#!/usr/bin/env python
# -*- coding: utf-8 -*-
from ctypes import *
import os

abs_file = os.path.abspath(os.path.dirname(__file__))
if -1 == abs_file.find('install'):
    lib_name = abs_file + "/../../../../devel/lib/" + "librlog.so"
else:
    lib_name = abs_file + "/../" + "librlog.so"
try:
    library = cdll.LoadLibrary(lib_name)
except:
    library = cdll.LoadLibrary("librlog.so")


class rlog:

    def setModuleName(self, name):
        library.set_modulename(name)

    def set_priority(self, level):
        library.set_priority(level)

    def info(self, msg):
        library.info(msg)

    def debug(self, msg):
        library.debug(msg)

    def notice(self, msg):
        library.notice(msg)

    def warn(self, msg):
        library.warn(msg)

    def error(self, msg):
        library.error(msg)

    def fatal(self, msg):
        library.fatal(msg)
