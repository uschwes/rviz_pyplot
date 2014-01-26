import roslib
import numpy as np 
from sensor_msgs.msg import Image 
from PlotObject import PlotObject

class ImageMarker(PlotObject):
    """An object representing an image to be plotted.
    """
    def __init__(self, frameId=None, topic=None):
        self._topic = topic
        self._frameId = frameId
        self._image = None

    def addImage(self, image):
        self._image = np.asarray(image, dtype=np.uint8)
        
    def appendMessages(self, stamp, messages):
        if not self._image is None:
            # Build the image message and push it on the message list
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frameId
            msg.width = self._image.shape[0]
            msg.height = self._image.shape[1]
            if (len(self._image.shape) == 2) or (len(self._image.shape) == 3 and self._image.shape[2] == 1):
                # A gray image
                msg.encoding = '8UC1'
                stepMult = 1
            elif len(self._image.shape) == 3 and self._image.shape[2] == 3:
                # A color image
                msg.encoding = 'rgb8'
                stepMult = 3
            elif len(self._image.shape) == 3 and self._image.shape[2] == 4:
                # A color image
                msg.encoding = 'rgba8'
                stepMult = 3
            else:
                raise RuntimeError("The parsing of images is very simple. " +\
                                   "Only 3-channel rgb (rgb8), 4 channel rgba " +\
                                   "(rgba8) and 1 channel mono (mono8) are " +\
                                   "supported. Got an image with shape " +\
                                   "{0}".format(self._image.shape))
            msg.is_bigendian = False
            msg.step = stepMult * msg.width
            msg.data = self._image.flatten().tolist()
            messages.append((self._topic, msg))
            
