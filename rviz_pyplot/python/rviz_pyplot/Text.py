from PlotObject import PlotObject
from Markers import initMarker
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import itertools
import numpy as np

class TextMarker(PlotObject):
    """Draw text in rviz
    """
    
    def __init__(self, frameId=None, topic=None, markerNamespace=None, markerId=0, scale=0.1):
        """Initialize the coordinate frames marker
        This marker displays text in a 3D spot in the world. The text always appears oriented correctly to the view. Uses the text field in the marker.

        Only scale.z is used. scale.z specifies the height of an uppercase "A".
        """
        self._topic = topic
        self._markerId = markerId
        self._frameId = frameId
        self._scale = scale
        
        if markerNamespace is None:
            self._markerNamespace = "/text"
        else:
            self._markerNamespace = markerNamespace

        self._text = None
        self._position = np.zeros(3)

    def setText(self, text, position):
        self._text = text
        self._position = position
        
    def buildMessage(self, stamp):
        text = initMarker(baseFrameId=self._frameId, 
                          markerNamespace=self._markerNamespace, 
                          markerId=self._markerId, 
                          stamp=stamp, 
                          markerType=Marker.TEXT_VIEW_FACING)
        text.scale.z = self._scale

        text.pose.position.x = self._position[0]
        text.pose.position.y = self._position[1]
        text.pose.position.z = self._position[2]
        
        # This next line is awesome
        text.text = self._text
        return text

    def appendMessages(self, stamp, messages):
        messages.append( (self._topic, self.buildMessage(stamp)) )
