from PlotObject import PlotObject
from Markers import initMarker
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import itertools
import numpy as np
import IPython

class EdgeListMarker(PlotObject):
    """A super class that will draw edges that can be sent to rviz
    """
    
    def __init__(self, frameId = None, topic=None, markerNamespace=None, markerId=0, defaultColor=None, lineWidth=0.01):
        """Initialize the coordinate frames marker
        """
        self._topic = topic
        self._markerId = markerId
        self._lineWidth = lineWidth
        self._frameId = frameId

        if defaultColor is None:
            self._defaultColor = ColorRGBA(*np.array([0.5,0.5,0.5,1.0]))
        else:
            self._defaultColor = defaultColor

        if markerNamespace is None:
            self._markerNamespace = "/edges"
        else:
            self._markerNamespace = markerNamespace

        self._edgePointPairs = []

    def addEdges(self, Ts):
        points = [Point(*p) for p in [T[0:3, 3] for T in Ts]]
        pairs = zip(points[:-1], points[1:])
        self._edgePointPairs.extend(pairs)

    def buildMessage(self, stamp):
        cframes = initMarker(baseFrameId=self._frameId, markerNamespace=self._markerNamespace, markerId=self._markerId, stamp=stamp, markerType=Marker.LINE_LIST)
        cframes.scale.x = self._lineWidth

        for pr in self._edgePointPairs:
            cframes.points.append(pr[0])
            cframes.points.append(pr[1])
            cframes.colors.append(self._defaultColor)
            cframes.colors.append(self._defaultColor)

        return cframes

    def appendMessages(self, stamp, messages):
        messages.append( (self._topic, self.buildMessage(stamp)) )
