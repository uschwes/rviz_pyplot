import rospy
import visualization_msgs
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import numpy as np


def initMarker(baseFrameId="world", stamp=None, markerNamespace=None, markerId=0, markerType=None, markerAction=Marker.ADD, markerDuration=rospy.Duration()):
    marker = Marker()
    # 	cframes.header.frame_id = baseFrameId;
    marker.header.frame_id = baseFrameId
    # 	marker.header.stamp = timestamp; // TODO: check on this...not sure this is right
    if stamp is None:
        stamp = rospy.Time.now()
    marker.header.stamp = stamp
    # 	marker.ns = "/vertices";
    if not markerNamespace is None:
        marker.ns = markerNamespace
    # 	marker.id = 0;
    marker.id = markerId
    # 	marker.type = visualization_msgs::Marker::LINE_LIST;
    if not markerType is None:
        marker.type = markerType
    # 	marker.action = visualization_msgs::Marker::ADD;
    marker.action = markerAction
    # 	marker.lifetime = ros::Duration();
    marker.lifetime = markerDuration
    # 	marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0;
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    # 	marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0;
    # 	marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    # 	marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    # 	marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.f;
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    # 	marker.frame_locked = false;
    marker.frame_locked = False
    return marker
