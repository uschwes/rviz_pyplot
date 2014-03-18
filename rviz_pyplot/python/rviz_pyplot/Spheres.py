from PlotObject import PlotObject
from Markers import initMarker
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import itertools
import numpy as np
import collections
import IPython

class SpheresMarker(PlotObject):
    """A super class that will draw lines that can be sent to rviz
    """
    
    def __init__(self, frameId=None, topic=None, markerNamespace=None, markerId=0, defaultColor=None, radius=1.0):
        """Initialize the coordinate frames marker
        """
        self._topic = topic
        self._markerId = markerId
        self._frameId = frameId
        self._radius = radius
        
        if defaultColor is None:
            self._defaultColor = np.array([0.3,0.3,0.9,1.0])
        else:
            self._defaultColor = defaultColor
        
        if markerNamespace is None:
            self._markerNamespace = "/lines"
        else:
            self._markerNamespace = markerNamespace

        self._centers = collections.deque()
        self._colors = collections.deque()


    def addSphere(self, p, color=None):
        if not (len(p.shape) == 1 and p.shape[0] == 3):
            raise RuntimeError("p1 must be a 3 vector. Got {0}".format(p1.shape))

        self._centers.appendleft(p)
        if color is None:
            self._colors.appendleft(self._defaultColor)
        else:
            self._colors.appendleft(color)

    def clear():
        self._centers.clear()
        self._colors.clear()

      
            
    def buildMessage(self, stamp):
        spheres = initMarker(baseFrameId=self._frameId, markerNamespace=self._markerNamespace, markerId=self._markerId, stamp=stamp, markerType=Marker.SPHERE_LIST)
        spheres.scale.x = self._radius

        for c, color in itertools.izip(self._centers, self._colors):
            spheres.points.append(Point(*c))
            spheres.colors.append(ColorRGBA(*color))

        
        return spheres

    def appendMessages(self, stamp, messages):
        messages.append( (self._topic, self.buildMessage(stamp)) )


        
        
