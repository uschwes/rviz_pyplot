import rospy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import PointCloud
reload(PointCloud)
import transformations
reload(transformations)    

from PointCloud import PointCloudMarker
from PlotObject import PlotObject  
from CoordinateFrames import CoordinateFramesMarker

class Plotter(object):
    def __init__(self, initRosNode=True, rosNodeName=None, visFrame=None):
        if initRosNode:
            if rosNodeName is None:
                rosNodeName = 'rviz_pyplot'#_%s'.format(uuid.uuid1().get_hex())
            rospy.init_node(rosNodeName)
        
        if visFrame is None:
            visFrame = "/rviz_pyplot"
        self._visFrame = visFrame

        # Create a map for publishers
        self._pointCloudPubs = {}
        self._markerArrayPubs = {}
        
        # \todo publish transforms in a thread.

    def __del__(self):
        # \todo clean up ROS
        pass
    
    def clf(self):
        """Clears all active messages"""
        pass

    def createPointCloud( self, points=None, visFrame = None, topic = None):
        """Creates and returns a point cloud object"""
        if visFrame is None:
            visFrame = self._visFrame
        if topic is None:
            topic = "{0}/points".format(rospy.get_name())
        return PointCloudMarker(visFrame, topic, points)
    
    def getPointCloudPublisher(self, topic=None):
        if topic is None:
            topic = "{0}/points".format(rospy.get_name())

        if topic in self._pointCloudPubs:
            pub = self._pointCloudPubs[topic]
        else:
            # Initialize a new publisher
            pub = rospy.Publisher(topic, PointCloud2, latch=True)
            # Save the publisher for later
            self._pointCloudPubs[topic] = pub

        return pub

    def getMarkerArrayPublisher(self, topic=None):
        if topic is None:
            topic = "{0}/points".format(rospy.get_name())

        if topic in self._markerArrayPubs:
            pub = self._markerArrayPubs[topic]
        else:
            # Initialize a new publisher
            pub = rospy.Publisher(topic, MarkerArray, latch=True)
            # Save the publisher for later
            self._markerArrayPubs[topic] = pub

        return pub

    def plot( self, plotItems ):
        stamp = rospy.Time.now()
        pointClouds = []
        markers = []
        if issubclass(type(plotItems), PlotObject):
            plotItems.appendMessages(stamp, pointClouds, markers)
            
        else:
            for item in plotItems:
                item.appendMessages(pointClouds, markers)
        
        for cloud in pointClouds:
            pub = self.getPointCloudPublisher(cloud[0])
            cloud[1].header.stamp = stamp
            print "publishing {0}!".format(cloud[0])
            #print cloud[1]
            pub.publish( cloud[1] )
