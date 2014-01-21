import roslib
import numpy as np 
from sensor_msgs.msg import PointCloud2, PointField 
from PlotObject import PlotObject
roslib.load_manifest('cv_bridge')
import cv_bridge

class ImageMarker(PlotObject):
    """An object representing an image to be plotted.
    """
    def __init__(self, frameId=None, topic=None):
        
