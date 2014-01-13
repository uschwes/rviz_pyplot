import roslib
import numpy as np 
from sensor_msgs.msg import PointCloud2, PointField 
from PlotObject import PlotObject
import recfunctions
reload(recfunctions)
dtypeMap={}
dtypeMap[np.float32] = PointField.FLOAT32
dtypeMap[np.dtype('float32')] = PointField.FLOAT32
dtypeMap[np.float64] = PointField.FLOAT64
dtypeMap[np.dtype('float64')] = PointField.FLOAT64
dtypeMap[np.int16] = PointField.INT16
dtypeMap[np.dtype('int16')] = PointField.INT16
dtypeMap[np.int8] = PointField.INT8
dtypeMap[np.dtype('int8')] = PointField.INT8
dtypeMap[np.int32] = PointField.INT32
dtypeMap[np.dtype('int32')] = PointField.INT32
dtypeMap[np.uint8] = PointField.UINT8
dtypeMap[np.dtype('uint8')] = PointField.UINT8
dtypeMap[np.uint16] = PointField.UINT16
dtypeMap[np.dtype('uint16')] = PointField.UINT16
dtypeMap[np.uint32] = PointField.UINT32
dtypeMap[np.dtype('uint32')] = PointField.UINT32
 
class PointCloudMarker(PlotObject):
    """An object representing a point cloud to be plotted.
    """
    
    def __init__(self, frameId, topic, points=None, rgbList=None, intensityList=None, namedChannels=None):
        """Initializes the point cloud object
        """

        self._frameId = frameId
        self._numPoints = 0
        self._topic = topic
        self.resetChannels()
        self.setPoints(points)
        self.setRgb(rgbList)
        self.setIntensity(intensityList)
        self.setNamedChannels(namedChannels)

    def resetChannels( self ):
        self._channels = {}
        self._channelOrder = []

    def setPoints( self, points ):
        if not points is None:
            S = points.shape
            if (not len(S) == 2) or (not S[1] == 3):
                raise RuntimeError("Expected an Nx3 array. Got {0}".format(S))

            if not len(points) == self._numPoints:
                self.resetChannels()
                self._numPoints = len(points)
            self.setNamedChannels( [('x',np.asarray(points[:,0], np.float32)),
                                    ('y',np.asarray(points[:,1], np.float32)),
                                    ('z',np.asarray(points[:,2], np.float32))] )

    def setRgb(self, rgbList):
        if not rgbList is None:
            S = rgbList.shape
            if not (len(S) == 2 and S[0] == self._numPoints and S[1] == 3):
                raise RuntimeError("The rgbList must be an Nx3 vector (color rgb). Shape was {0}".format(S))
            
            self.setNamedChannels( [('r',np.asarray(rgbList[:,0], np.float32)),
                                    ('g',np.asarray(rgbList[:,1], np.float32)),
                                    ('b',np.asarray(rgbList[:,2], np.float32))] )

    def setIntensity(self, intensityList):
        if not intensityList is None:
            S = intensityList.shape
            if not len(intensityList.shape) == 1:
                intensityList = intensityList.flatten()
            if not (len(intensityList) == self._numPoints):
                raise RuntimeError("The intensityList must be an N-length-vector (intensity). Shape was {0}".format(S))
            self.setNamedChannels( [('intensity',np.asarray(intensityList, np.float32))] )
    
    def setNamedChannels( self, channelList ):
        if not channelList is None:
            for channel in channelList:
                if not channel[0] in self._channels:
                    self._channelOrder.append(channel[0])
                if not len(channel[1]) == self._numPoints:
                    raise RuntimeError("Channel {0} was an unexpected size. Expected {1}, got {2}".format(channel[0], self._numPoints, len(channel[1])))
                self._channels[channel[0]] = channel[1]

    def buildMessage(self, stamp):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frameId
        msg.is_bigendian = False 
        msg.is_dense = True
        msg.height = 1
        msg.width = self._numPoints
        msg.fields = []
        arrays = []
        channelStart=0
        for channelName in self._channelOrder:
            arr = self._channels[channelName]
            msg.fields.append( PointField( channelName, channelStart, dtypeMap[arr.dtype],1) )
            arrays.append(arr)
            channelStart = channelStart + arr.dtype.itemsize
    
        msg.data = recfunctions.merge_arrays(arrays).tostring()
        msg.point_step=channelStart
        msg.row_step=channelStart * self._numPoints
        return msg
    def appendMessages(self, stamp, pointClouds, markers):
        pointClouds.append( (self._topic, self.buildMessage(stamp)) )
