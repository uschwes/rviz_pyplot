import rospy
import uuid
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

# void addCFrameToLineList(std::vector<geometry_msgs::Point> & points, std::vector<std_msgs::ColorRGBA> & colors, const Eigen::Matrix4d & T_0_k, double size)
# {
#   std_msgs::ColorRGBA col[3];
#   // red
#   col[0].a = col[0].r = 1.0; col[0].g = col[0].b = 0.0;
#   // green
#   col[1].a = col[1].g = 1.0; col[1].r = col[1].b = 0.0;
#   // blue
#   col[2].a = col[2].b = 1.0; col[2].g = col[2].r = 0.0;


#   // Draw a coordinate frame.
#   Eigen::Vector3d p_0_k_0 = T_0_k.topRightCorner<3,1>();
#   for(int i = 0; i < 3; i++)
#     {
#       points.push_back(toPointMessage(p_0_k_0));
#       colors.push_back(col[i]);

#       points.push_back(toPointMessage(p_0_k_0 + (T_0_k.col(i).head<3>() * size)));
#       colors.push_back(col[i]);     
#     } 
# }

def buildLineMarker(baseFrameId, P, stamp, markerNamespace, lineWidth, markerId=0, intensity=None, intensityList=None, rgb=None, rgba=None, rgbList=None, rgbaList=None):
    if (not len(P.shape) == 2) or (not P.shape[1] == 3):
        raise RuntimeError("Expected P to be an Nx3 array of points. P was {0}".format(P.shape))
    line = initMarker( baseFrameId = baseFrameId, stamp = stamp, markerNamespace = markerNamespace, markerType = Marker.LINE_STRIP, markerId = markerId )
    # http://wiki.ros.org/rviz/DisplayTypes/Marker#Line_Strip_.28LINE_STRIP.3D4.29
    line.scale.x = lineWidth

    isSingleColor = True
    lineCol = ColorRGBA(1.0,1.0,1.0,1.0)

    if not rgbaList is None:
        # Check the size
        if (not len(rgbaList.shape) == 2) or (not rgbaList.shape[0] == len(P)) or (not rgbaList.shape[1] == 4):
            raise RuntimeError("Expected rgbaList (shape {0}) to be Nx4. N={1}".format(rgbaList.shape,P.shape[0]))
        for i in range(len(P)):
            line.colors.append(ColorRGBA(rgbaList[i,0],rgbaList[i,1],rgbaList[i,2],rgbaList[i,3]))
    elif not rgbList is None:
        # Check the size
        if (not len(rgbList.shape) == 2) or (not rgbList.shape[0] == len(P)) or (not rgbList.shape[1] == 3):
            raise RuntimeError("Expected rgbList (shape {0}) to be Nx3. N={1}".format(rgbList.shape,P.shape[0]))
        for i in range(len(P)):
            line.colors.append(ColorRGBA(rgbList[i,0],rgbList[i,1],rgbList[i,2],1.0))
    elif not intensityList is None:
        # Check the size
        if (not len(intensityList.shape) == 1) or (not intensityList.shape[0] == len(P)):
            raise RuntimeError("Expected intensityList (shape {0}) to be an N-vector. N={1}".format(intensityList.shape,P.shape[0]))
        for i in range(len(P)):
            line.colors.append(ColorRGBA(intensityList[i],intensityList[i],intensityList[i],1.0))
    elif not rgba is None:
        for i in range(len(P)):
            line.colors.append(ColorRGBA(rgba[0],rgba[1],rgba[2],rgba[3]))
    elif not rgb is None:
        for i in range(len(P)):
            line.colors.append(ColorRGBA(rgb[0],rgb[1],rgb[2],1.0))
    elif not intensity is None:
        for i in range(len(P)):
            line.colors.append(ColorRGBA(intensity,intensity,intensity,1.0))
    else:
        for i in range(len(P)):
            line.colors.append(lineCol)

    for i in range(len(P)):
        line.points.append(Point(P[i,0],P[i,1],P[i,2]))
    return line

                       
    

def buildMarkerArrayFromPoses(baseFrameId, TT, stamp=None, frameSize=0.1, frameLineWidth=0.01, lineWidth=0.01, doPath=True, pathColor=None, frameColors=None, markerId=0, baseNamespace=None):
    # visualization_msgs::MarkerArray buildMaFromPoses(const std::string & baseFrameId, ros::Time timestamp, std::vector<geometry_msgs::Pose>& poses, double cframeSize)
    # {
    # 	visualization_msgs::MarkerArray ma;
    if stamp is None:
        stamp = rospy.Time.now()
    ma = MarkerArray()

    if baseNamespace is None:
        baseNamespace = ''

    # Create the path.
    if doPath:
        path = np.vstack([T[0:3,3] for T in TT])
        if pathColor == None:
            pathColor = np.array([0.5,0.5,0.8,1.0])
        lines = buildLineMarker(baseFrameId, path, stamp, '{0}/path'.format(baseNamespace), lineWidth, rgba=pathColor, markerId=markerId)

        ma.markers.append(lines)

    cframes = initMarker(baseFrameId = baseFrameId, stamp = stamp, markerNamespace = "{0}/coordinateFrames".format(baseNamespace), markerType = Marker.LINE_LIST, markerId = markerId)
    cframes.scale.x = frameLineWidth

    if frameColors is None:
        frameColors = np.array( [[1,0,0,1],[0,1,0,1],[0,0,1,1]] )

    xcolor = ColorRGBA(*frameColors[0,:])
    ycolor = ColorRGBA(*frameColors[1,:])
    zcolor = ColorRGBA(*frameColors[2,:])
    for T in TT:
        origin = T[0:3,3]
        porigin = Point(*origin)
        xaxis = Point(*(origin + T[0:3,0] * frameSize))
        yaxis = Point(*(origin + T[0:3,1] * frameSize))
        zaxis = Point(*(origin + T[0:3,2] * frameSize))
        
        cframes.points.append(porigin)
        cframes.points.append(xaxis)
        cframes.colors.append(xcolor)
        cframes.colors.append(xcolor)

        cframes.points.append(porigin)
        cframes.points.append(yaxis)
        cframes.colors.append(ycolor)
        cframes.colors.append(ycolor)

        cframes.points.append(porigin)
        cframes.points.append(zaxis)
        cframes.colors.append(zcolor)
        cframes.colors.append(zcolor)

    ma.markers.append(cframes)
    return ma
