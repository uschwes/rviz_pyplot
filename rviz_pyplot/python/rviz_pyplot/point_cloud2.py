import roslib
import numpy as np 
from sensor_msgs.msg import PointCloud2, PointField 

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
 

def buildPointCloud2(points, stamp=None, frame_id=None, rgbList=None, intensityList=None, namedChannels=None): 
    msg = PointCloud2() 
    if stamp: 
        msg.header.stamp = stamp 
    if frame_id: 
        msg.header.frame_id = frame_id 
    if len(points.shape) == 3: 
        msg.height = points.shape[1] 
        msg.width = points.shape[0] 
    else: 
        msg.height = 1 
        msg.width = len(points) 
    
    channels = [];
    channels.append( ('x', np.asarray(points[:,0],np.float32) ) )
    channels.append( ('y', np.asarray(points[:,1],np.float32) ) )
    channels.append( ('z', np.asarray(points[:,2],np.float32) ) )

    if not rgbList is None:
        S = rgbList.shape
        if not (len(S) == 2 and S[0] == len(points) and S[1] == 3):
            raise RuntimeError("The rgbList must be an Nx3 vector (color). Shape was {0}".format(S))
        channels.append( ('r', np.asarray(rgbList[:,0],np.float32) ) )
        channels.append( ('g', np.asarray(rgbList[:,1],np.float32) ) )
        channels.append( ('b', np.asarray(rgbList[:,2],np.float32) ) )

    elif not intensityList is None:
        S = intensityList.shape
        if not (len(S) == 1 and S[0] == len(points)):
            raise RuntimeError("The intensityList must be an N vector. Shape was {0}".format(S))
        channels.append( ('intensity', np.asarray(intensityList,np.float32) ) )

    if not namedChannels is None:
        channels = channels + namedChannels

    msg.is_bigendian = False 
    msg.row_step = msg.point_step*points.shape[0] 
    msg.is_dense = int(np.isfinite(points).all()) 
    msg.fields = []
    arrays = []
    channelStart=0
    for channel in channels:
        msg.fields.append( PointField( channel[0], channelStart, dtypeMap[channel[1].dtype],1) )
        arrays.append(channel[1])
        channelStart = channelStart + channel[1].dtype.itemsize
    
    msg.data = np.lib.recfunctions.merge_arrays(arrays).tostring()
    msg.point_step=channelStart
    msg.row_step=channelStart * len(points)
    
    return msg 
