import rospy
import uuid
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import point_cloud2
reload(point_cloud2)
import transformations
reload(transformations)

class Plotter(object):
    def __init__(self, initRosNode=True, rosNodeName=None, visFrame=None):
        if initRosNode:
            if rosNodeName is None:
                rosNodeName = 'rviz_plotting_node'#_%s'.format(uuid.uuid1().get_hex())
            rospy.init_node(rosNodeName)
        
        if visFrame is None:
            visFrame = "/rviz_plotting"
        self._visFrame = visFrame

        # Create a map for publishers
        self._pointCloudPubs = {}
        self._markerArrayPubs = {}
        
        # \todo publish transforms in a thread.

    def __del__(self):
        # \todo clean up ROS
        pass

    # Make member functions after development
    def plot3( self, P, topic=None, rgbList=None, intensityList=None, namedChannels=None):
        """plot a point cloud. P should be Nx3
    color can be a Nx1 (intensity) or an Nx3 (color) matrix"""
        if topic is None:
            topic = "{0}/points".format(rospy.get_name())

        if topic in self._pointCloudPubs:
            pub = self._pointCloudPubs[topic]
        else:
            # Initialize a new publisher
            pub = rospy.Publisher(topic, PointCloud2, latch=True)
            # Save the publisher for later
            self._pointCloudPubs[topic] = pub

        S = P.shape
        if (not len(S) == 2) or (not S[1] == 3):
            raise RuntimeError("The point cloud must be an Nx3 matrix. Shape was {0}".format(S))

        msg = point_cloud2.buildPointCloud2(P, rgbList=None, intensityList=None, stamp=rospy.Time.now(), frame_id=self._visFrame, namedChannels=namedChannels) 
        pub.publish(msg)

    def plotTrajectory( self, trajectory, topic=None, baseNamespace=None, markerId=0):
        """plot a trajectory. 
    trajectory is an ordered list of transformation matrices representing T_world_frame
    """
        if topic is None:
            topic = "{0}/trajectory".format(rospy.get_name())

        if topic in self._markerArrayPubs:
            pub = self._markerArrayPubs[topic]
        else:
            # Initialize a new publisher
            pub = rospy.Publisher(topic, MarkerArray, latch=True)
            # Save the publisher for later
            self._markerArrayPubs[topic] = pub

        stamp = rospy.Time.now()
        ma = transformations.buildMarkerArrayFromPoses(self._visFrame, trajectory, stamp=stamp, baseNamespace=baseNamespace, markerId=markerId)
        pub.publish(ma)

    def plotCoordinateFrames( self, frames, frameIds=None, topic=None, baseNamespace=None, markerId=0):
        """plot a trajectory. 
    trajectory is an ordered list of transformation matrices representing T_world_frame
    """
        if topic is None:
            topic = "{0}/frames".format(rospy.get_name())

        if topic in self._markerArrayPubs:
            pub = self._markerArrayPubs[topic]
        else:
            # Initialize a new publisher
            pub = rospy.Publisher(topic, MarkerArray, latch=True)
            # Save the publisher for later
            self._markerArrayPubs[topic] = pub

        stamp = rospy.Time.now()
        ma = transformations.buildMarkerArrayFromPoses(self._visFrame, trajectory, stamp=stamp, baseNamespace=baseNamespace, markerId=markerId, doPath=False)
        
        
        
        pub.publish(ma)

    
    def plotLine( self, P, topic=None):
        """plot a line. P is an Nx3 array of points"""
        pass

    def plotLines( self, P, topic=None):
        """plot lines. P is a list of lines where each element is an Nx3 array of points"""
        pass

    def plotCoordinateFrames( self, T, topic=None ):
        """plot a series of coordinate frames (unconnected).
    T is a list of 4x4 transformation matrices representing T_world_frame"""
        pass


