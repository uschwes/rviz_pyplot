from PlotObject import PlotObject
from Markers import initMarker
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import itertools
import numpy as np
import collections
import IPython

class TrianglesMarker(PlotObject):
    """A super class that will draw lines that can be sent to rviz
    """
    
    def __init__(self, frameId=None, topic=None, markerNamespace=None, markerId=0, defaultColor=None, history=None):
        """Initialize the coordinate frames marker
        """
        self._topic = topic
        self._markerId = markerId
        self._frameId = frameId
        self._history = history
        
        if defaultColor is None:
            self._defaultColor = np.array([0.3,0.3,0.9,1.0])
        else:
            self._defaultColor = defaultColor
        
        if markerNamespace is None:
            self._markerNamespace = "/triangles"
        else:
            self._markerNamespace = markerNamespace

        self._points = collections.deque()
        self._colors = collections.deque()
        self._times = collections.deque()


    def addTriangle(self, p1, p2, p3, color=None, time=0):
        if not (len(p1.shape) == 1 and p1.shape[0] == 3):
            raise RuntimeError("p1 must be a 3 vector. Got {0}".format(p1.shape))
        if not (len(p2.shape) == 1 and p2.shape[0] == 3):
            raise RuntimeError("p2 must be a 3 vector. Got {0}".format(p2.shape))
	if not (len(p3.shape) == 1 and p3.shape[0] == 3):
            raise RuntimeError("p3 must be a 3 vector. Got {0}".format(p3.shape))
        self._points.appendleft(p1)
        self._points.appendleft(p2)
        self._points.appendleft(p3)
        if color is None:
            self._colors.appendleft(self._defaultColor)
            self._colors.appendleft(self._defaultColor)
            self._colors.appendleft(self._defaultColor)
        else:
            self._colors.appendleft(color)
            self._colors.appendleft(color)
            self._colors.appendleft(color)

        self._times.appendleft(time)
        self._times.appendleft(time)
        self._times.appendleft(time)

      
        if self._history is not None:
            if False and self._history > 0: #alpha is not used by the line marker (yet) :-(
                # adjust alpha
                n = len(self._colors)
                for i in range(n):
                    t = self._times[i]
                    age = time - t
                    age = 1.0 - (age / self._history)
                    age = self._colors[i][3] * age
                    self._colors[i][3] = age

            while self._times[-1] < time - self._history:
                self._times.pop()
                self._times.pop()
                self._times.pop()
                self._points.pop()
                self._points.pop()
                self._points.pop()
                self._colors.pop()
                self._colors.pop()
                self._colors.pop()
        
    def buildMessage(self, stamp):
        triangles = initMarker(baseFrameId=self._frameId, markerNamespace=self._markerNamespace, markerId=self._markerId, stamp=stamp, markerType=Marker.TRIANGLE_LIST)

        for p, color in itertools.izip(self._points, self._colors):
            triangles.points.append(Point(*p))
            triangles.colors.append(ColorRGBA(*color))
            
            # There seems to a be a bug so that the alpha channel for every object is taken from to global color entry
            triangles.color = ColorRGBA(*color)

        return triangles

    def appendMessages(self, stamp, messages):
        messages.append( (self._topic, self.buildMessage(stamp)) )


        
        
