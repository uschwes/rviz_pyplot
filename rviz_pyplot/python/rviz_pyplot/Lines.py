from PlotObject import PlotObject
from Markers import initMarker
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import itertools
import numpy as np

class LinesMarker(PlotObject):
    """A super class that will draw lines that can be sent to rviz
    """
    
    def __init__(self, frameId=None, topic=None, markerNamespace=None, markerId=0, defaultColor=None, lineWidth=0.01):
        """Initialize the coordinate frames marker
        """
        self._topic = topic
        self._markerId = markerId
        self._frameId = frameId
        self._lineWidth = lineWidth
        
        if defaultColor is None:
            self._defaultColor = np.array([0.3,0.3,0.9,1.0])
        else:
            self._defaultColor = defaultColor
        
        if markerNamespace is None:
            self._markerNamespace = "/lines"
        else:
            self._markerNamespace = markerNamespace

        self._points = []
        self._colors = []

    def addLine(self, p1, p2, color=None):
        if not (len(p1.shape) == 1 and p1.shape[0] == 3):
            raise RuntimeError("p1 must be a 3 vector. Got {0}".format(p1.shape))
        if not (len(p2.shape) == 1 and p2.shape[0] == 3):
            raise RuntimeError("p1 must be a 3 vector. Got {0}".format(p2.shape))
        self._points.append(p1)
        self._points.append(p2)
        if color is None:
            self._colors.append(self._defaultColor)
            self._colors.append(self._defaultColor)
        else:
            self._colors.append(color)
            self._colors.append(color)
    
    def addLineStrip(self, P, color=None):
        S = P.shape
        if (not len(S) == 2) or (not S[1] == 3):
            raise RuntimeError("The point list must be Nx3. Got {0}".format(S))
        for row in range(1,S[0]):
            self.addLine(P[row-1,:],P[row,:],color)
        
    def buildMessage(self, stamp):
        lines = initMarker(baseFrameId=self._frameId, markerNamespace=self._markerNamespace, markerId=self._markerId, stamp=stamp, markerType=Marker.LINE_LIST)
        lines.scale.x = self._lineWidth

        for p, color in itertools.izip(self._points, self._colors):
            lines.points.append(Point(*p))
            lines.colors.append(ColorRGBA(*color))

        return lines

    def appendMessages(self, stamp, messages):
        messages.append( (self._topic, self.buildMessage(stamp)) )


        
        
