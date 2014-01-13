from PlotObject import PlotObject
from Markers import initMarker
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import itertools
import numpy as np

class CoordinateFramesMarker(PlotObject):
    """A super class that will draw coordinate frames that can be sent to rviz
    """
    
    def __init__(self, frameId=None, topic=None, markerNamespace=None, markerId=0, defaultSize=0.1, defaultColors=None, lineWidth=0.01):
        """Initialize the coordinate frames marker
        """
        self._topic = topic
        self._markerId = markerId
        self._frameId = frameId
        self._defaultSize = defaultSize
        self._lineWidth = lineWidth
        
        if defaultColors is None:
            self._defaultColors = np.eye(3,4)
            self._defaultColors[:,3] = np.ones(3)
        else:
            self._defaultColors = defaultColors
        
        if markerNamespace is None:
            self._markerNamespace = "/axes"
        else:
            self._markerNamespace = markerNamespace

        self._frames = []
        self._colorsList = []
        self._sizeList = []

    def addCoordinateFrame(self, T, colors=None, size=None):
        self._frames.append(T)
        if colors is None:
            self._colorsList.append(self._defaultColors)
        else: 
            self._colorsList.append(colors)
        if size is None:
            self._sizeList.append(self._defaultSize)
        else: 
            self._sizeList.append(size)

    def addCoordinateFrames(self, T, colorsList=None, sizeList=None):
        self._frames.extend(T)
        if colorsList is None:
            self._colorsList.extend( [self._defaultColors]*len(T) )
        else:
            if not len(colorsList) == len(T):
                raise RuntimeError("Expected the list of colors (length {0}) to be the same length as the list of transformations (length {1})".format(len(colorsList), len(T)))
            self._colorsList.extend( colorsList )
        if sizeList is None:
            self._sizeList.extend( [self._defaultSize]*len(T) )
        else:
            if not len(sizeList) == len(T):
                raise RuntimeError("Expected the list of sizes (length {0}) to be the same length as the list of transformations (length {1})".format(len(sizeList), len(T)))
            self._sizeList.extend( sizeList )
        
    def buildMessage(self, stamp):
        cframes = initMarker(baseFrameId=self._frameId, markerNamespace=self._markerNamespace, markerId=self._markerId, stamp=stamp, markerType=Marker.LINE_LIST)
        cframes.scale.x = self._lineWidth

        for T, colors, frameSize in itertools.izip(self._frames, self._colorsList, self._sizeList):
            origin = T[0:3,3]
            porigin = Point(*origin)
            xaxis = Point(*(origin + T[0:3,0] * frameSize))
            yaxis = Point(*(origin + T[0:3,1] * frameSize))
            zaxis = Point(*(origin + T[0:3,2] * frameSize))
            xcolor = ColorRGBA(*colors[0,:])
            ycolor = ColorRGBA(*colors[1,:])
            zcolor = ColorRGBA(*colors[2,:])
        
            cframes.points.append(porigin)
            cframes.points.append(xaxis)
            cframes.colors.append(xcolor)
            cframes.colors.append(xcolor)

            cframes.points.append(porigin)
            cframes.points.append(yaxis)
            cframes.colors.append(ycolor)
            cframes.colors.append(ycolor)

            cframes.points.append(porigin)
            cframes.points.append(zaxis)
            cframes.colors.append(zcolor)
            cframes.colors.append(zcolor)

        return cframes
    def appendMessages(self, stamp, pointClouds, markers):
        markers.append( (self._topic, self.buildMessage(stamp)) )


        
        
