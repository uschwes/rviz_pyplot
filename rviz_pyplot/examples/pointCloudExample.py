import roslib
roslib.load_manifest('rviz_pyplot')
import rviz_pyplot as rpp
reload(rpp)
import numpy as np;

# Create a point cloud with N points
# The point cloud should be an Nx3 array
N = 10000
S = 10.0
Y = S * np.random.rand(N,3)


# Create a plotter
plotter = rpp.Plotter()

# Get a point cloud object from the plotter
# The topic is where the points will be published
pointCloud = rpp.PointCloudMarker(topic="/rviz_pyplot/points")

# Set the points.
pointCloud.setPoints(Y)

# Set the positions as RGB color values between 0 and 1
pointCloud.setRgb(np.abs(Y)/np.max(np.abs(Y.flatten())))

# Set the intensity as float values between 0 and 1
pointCloud.setIntensity( np.abs(Y[:,2])/np.max(np.abs(Y[:,1])))

# Named channels allow you to attach arbitrary data to points.
# Create some named channels. When you select the points in 
# rviz, you will see the values of the named channels.
namedChannels = []
# Named channels can be {int8, int16, int32, uint8, uint16, uint32, float32, float64}
namedChannels.append( ('height', np.asarray(Y[:,2], dtype=np.int8) ) )
namedChannels.append( ('blah', np.asarray(Y[:,1], dtype=np.int32) ) )
pointCloud.setNamedChannels( namedChannels )

# This sends the message to rviz
plotter.plot(pointCloud)
