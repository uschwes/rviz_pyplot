from PlotObject import PlotObject
from Markers import initMarker
from visualization_msgs.msg import MarkerArray,Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import itertools
import numpy as np
import collections
import IPython

import transformations as tr

class CubeMarker(PlotObject):
    """A super class that will draw lines that can be sent to rviz
    """

    def __init__(self, pose, scale=np.array([1.0,1.0,1.0]), frameId=None, topic=None, markerNamespace=None, markerId=0, color=None, history=None):
        """Initialize the coordinate frames marker
        """
        
        if not type(pose) is Pose:
          if not type(pose) is tuple:
            raise RuntimeError("Invalid pose input")
          else:
            position,orientation=pose
            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            if not type(orientation) is Quaternion:
              if len(orientation) == 4:
                pose.orientation.x = orientation[0]
                pose.orientation.y = orientation[1]
                pose.orientation.z = orientation[2]
                pose.orientation.w = orientation[3]
              #elif len(orientation) == 3:
                #quat = tr.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
              else:
                #raise RuntimeError("Invalid pose input, either specify a quaternion with 4 entries or euler angles yaw,pitch,roll")
                raise RuntimeError("Invalid pose input, either specify a quaternion with 4 entries")
            else:
              pose.orientation = orientation
            
        self._topic = topic
        self._markerId = markerId
        self._frameId = frameId
        self._history = history

        if color is None:
            self._color = np.array([0.3,0.3,0.9,1.0])
        else:
            self._color = color

        if markerNamespace is None:
            self._markerNamespace = "/cubes"
        else:
            self._markerNamespace = markerNamespace

        self._pose = pose
        self._scale = scale

    def buildMessage(self, stamp):
        cube = initMarker(baseFrameId=self._frameId, markerNamespace=self._markerNamespace, markerId=self._markerId, stamp=stamp, markerType=Marker.CUBE)
        cube.scale.x = self._scale[0]
        cube.scale.y = self._scale[1]
        cube.scale.z = self._scale[2]
        cube.pose = self._pose
        cube.color = ColorRGBA(*self._color)

        return cube

    def appendMessages(self, stamp, messages):
        messages.append( (self._topic, self.buildMessage(stamp)) )
