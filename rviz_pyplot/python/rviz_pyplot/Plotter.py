import rospy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import PointCloud
reload(PointCloud)
import CoordinateFrames
reload(CoordinateFrames)  
import Lines
reload(Lines)  

from PointCloud import PointCloudMarker
from PlotObject import PlotObject  
from CoordinateFrames import CoordinateFramesMarker

class Plotter(object):
    def __init__(self, initRosNode=True, rosNodeName=None, visFrame=None):
        if initRosNode:
            if rosNodeName is None:
                rosNodeName = 'rviz_pyplot'#_%s'.format(uuid.uuid1().get_hex())
            rospy.init_node(rosNodeName,['Plotter.py'])
        
        if visFrame is None:
            visFrame = "/rviz_pyplot"
        self._visFrame = visFrame
        self._defaultPointTopic = "{0}/points".format(rospy.get_name())
        self._defaultMarkerArrayTopic = "{0}/marker_array".format(rospy.get_name())

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
    
    def getDefaultPointCloudTopic(self):
        return self._defaultPointCloudTopic

    def getDefaultMarkerArrayTopic(self):
        return self._defaultMarkerArrayTopic

    def getPointCloudPublisher(self, topic=None):
        if topic is None:
            topic = self._defaultPointTopic

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
            topic = self._defaultMarkerArrayTopic

        if topic in self._markerArrayPubs:
            pub = self._markerArrayPubs[topic]
        else:
            # Initialize a new publisher
            pub = rospy.Publisher(topic, MarkerArray, latch=True)
            # Save the publisher for later
            self._markerArrayPubs[topic] = pub

        return pub

    def activeTopics( self ):
        return (self._pointCloudPubs.keys(), self._markerArrayPubs.keys())
    
    def printActiveTopics( self ):
        print "Point cloud topics:"
        for key in self._pointCloudPubs.keys():
            print "\t{0}".format(key)
        print "Marker array topics:"
        for key in self._markerArrayPubs.keys():
            print "\t{0}".format(key)

    def plot( self, plotItems ):
        stamp = rospy.Time.now()
        # Accumulate a list of point clouds and markers to publish
        pointClouds = []
        markers = []
        if type(plotItems) == list:
            for item in plotItems:
                item.appendMessages(pointClouds, markers)
        else:
            # Assume this is a single plotItem
            plotItems.appendMessages(stamp, pointClouds, markers)
        
        for topic, msg in pointClouds:
            pub = self.getPointCloudPublisher(topic)
            # Always override the stamp. This is a design choice
            # that may be revisited
            msg.header.stamp = stamp
            if msg.header.frame_id is None:
                msg.header.frame_id = self._visFrame
            pub.publish( msg )
        
        topics = {}
        for topic, msg in markers:
            msg.header.stamp = stamp
            if msg.header.frame_id is None:
                msg.header.frame_id = self._visFrame
            
            if topic in topics:
                topics[topic].markers.append(msg)
            else:
                ma = MarkerArray()
                ma.markers.append(msg)
                topics[topic] = ma
                
        for topic, ma in topics.iteritems():
            pub = self.getMarkerArrayPublisher(topic)
            pub.publish(ma)
    
