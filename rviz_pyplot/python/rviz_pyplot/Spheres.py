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

        self._spheres = collections.deque()


    def addSphere(self, p, color=None, time=0):
        if not (len(p.shape) == 1 and p.shape[0] == 3):
            raise RuntimeError("p1 must be a 3 vector. Got {0}".format(p1.shape))


        if color is None:
            color = self._defaultColor
        
        self._spheres.appendleft((p, color, time))

    def clearOlderThan(self, t):
        print "clearing spheres older than ", t
        print "num spheres in the list before: ", len(self._spheres)
        #tmp = collections.deque([(center, color, time) for center, color, time in self._spheres if time >= t])  
        self._spheres = collections.deque([(center, color, time) for center, color, time in self._spheres if time >= t])  
        print "num spheres in the list after: ", len(self._spheres)

    def clear(self):
        self._spheres.clear()

    def remove(self, p):
        self._spheres = collections.deque([(pnt, color, time) for pnt, color, time in self._spheres if not (pnt[0] == p[0] and pnt[1] == p[1] and pnt[2] == p[2])])
            
    def buildMessage(self, stamp):
        spheres = initMarker(baseFrameId=self._frameId, markerNamespace=self._markerNamespace, markerId=self._markerId, stamp=stamp, markerType=Marker.SPHERE_LIST)
        spheres.scale.x = self._radius

        for c, color, time in self._spheres:
            spheres.points.append(Point(*c))
            spheres.colors.append(ColorRGBA(*color))

        
        return spheres

    def appendMessages(self, stamp, messages):
        messages.append( (self._topic, self.buildMessage(stamp)) )


        
        
