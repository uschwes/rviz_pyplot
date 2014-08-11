import rospy
import PointCloud
reload(PointCloud)
import CoordinateFrames
reload(CoordinateFrames)  
import Lines
reload(Lines)
import Image as ImagePy
reload(ImagePy)
import Text
reload(Text)

from PointCloud import PointCloudMarker
from PlotObject import PlotObject  
from CoordinateFrames import CoordinateFramesMarker
from Image import ImageMarker

from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import Marker, MarkerArray


publishedMessages = []  # to be able to delete them later with clf

class Plotter(object):
    def __init__(self, initRosNode=True, rosNodeName=None, visFrame=None):
        if initRosNode:
            if rosNodeName is None:
                rosNodeName = 'rviz_pyplot'#_%s'.format(uuid.uuid1().get_hex())
            rospy.init_node(rosNodeName,['Plotter.py'], disable_signals=True)
        
        if visFrame is None:
            visFrame = "/rviz_pyplot"
        self._visFrame = visFrame

        self._publishers = {}
        self._publishers[PointCloud2] = {}
        self._publishers[MarkerArray] = {}
        self._publishers[Image] = {}

        self._defaultTopics = {}
        self._defaultTopics[PointCloud2] = "{0}/points".format(rospy.get_name())
        self._defaultTopics[MarkerArray] = "{0}/marker_array".format(rospy.get_name())
        self._defaultTopics[Image] = "{0}/images".format(rospy.get_name())

        # \todo publish transforms in a thread.

    def __del__(self):
        # \todo clean up ROS
        pass

    def clf(self):
	global publishedMessages
	for topic,msg in publishedMessages:
	    if type(msg) == Marker:
	      pub = self.getPublisher(Marker, topic)
	      msg.action = Marker.DELETE
	    if type(msg) == MarkerArray:
	      pub = self.getPublisher(MarkerArray, topic)
	      for m in msg.markers:
		m.action = Marker.DELETE
	    else:
		continue
	    pub.publish( msg )
	publishedMessages = []
    
    def getDefaultPointCloudTopic(self):
        return self._defaultPointCloudTopic

    def getDefaultMarkerArrayTopic(self):
        return self._defaultMarkerArrayTopic

    def getPublisher(self, messageType, topic=None, latch=True):
        publisherList = self._publishers[messageType]
        if topic is None:
            topic = self._defaultTopics[messageType]

        if topic in publisherList:
            pub = publisherList[topic]
        else:
            # Initialize a new publisher
            pub = rospy.Publisher(topic, messageType, latch=latch)
            # Save the publisher for later
            publisherList[topic] = pub

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

    def plot( self, plotItems, stamp=None ):
        if stamp is None:
            stamp = rospy.Time.now()
        # Accumulate a list of point clouds and markers to publish
        messages = []
        if type(plotItems) == list:
            for item in plotItems:
                item.appendMessages(stamp, messages)
        else:
            # Assume this is a single plotItem
            plotItems.appendMessages(stamp, messages)

        global publishedMessages
	
        topics = {}        
        for topic, msg in messages:
            if type(msg) == PointCloud2:
                pub = self.getPublisher(PointCloud2, topic)
                # Always override the stamp. This is a design choice
                # that may be revisited
                msg.header.stamp = stamp
                if msg.header.frame_id is None:
                    msg.header.frame_id = self._visFrame
                pub.publish( msg )
                publishedMessages.append( (topic,msg) )
            elif type(msg) == Marker:
                msg.header.stamp = stamp
                if msg.header.frame_id is None:
                    msg.header.frame_id = self._visFrame
                if topic in topics:
                    topics[topic].markers.append(msg)
                else:
                    ma = MarkerArray()
                    ma.markers.append(msg)
                    topics[topic] = ma
            elif type(msg) == Image:
                pub = self.getPublisher(Image, topic)
                # Always override the stamp. This is a design choice
                # that may be revisited
                msg.header.stamp = stamp
                if msg.header.frame_id is None:
                    msg.header.frame_id = self._visFrame
                pub.publish( msg )
                publishedMessages.append( (topic,msg) )
            else:
                raise RuntimeError("Unknown message type {0}\n{1}".format(type(msg), msg))
        for topic, ma in topics.iteritems():
            pub = self.getPublisher(MarkerArray, topic)
            pub.publish(ma)
            publishedMessages.append( (topic,ma) )

    def plotImage(self, I, frameId=None, topic=None):
        img = ImageMarker(frameId=frameId, topic=topic)
        img.addImage(I)
        self.plot(img)

    def plotText(self, text, position, scale, frameId=None, topic=None):
        textMarker = Text.TextMarker(frameId=frameId, topic=topic, scale=scale)
        textMarker.setText(text, position)
        self.plot(textMarker)
