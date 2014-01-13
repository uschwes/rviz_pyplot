import roslib
roslib.load_manifest('rviz_pyplot')
import rviz_pyplot as rpp
reload(rpp)
import numpy as np

import transformations as tr

# Create N coordinate frames
N = 100
scale=10

frames = []
for i in range(0,N):
    T = tr.random_rotation_matrix()
    T[0:3,3] = np.random.rand(3) * scale
    frames.append(T)

# Create a plotter
plotter = rpp.Plotter()

cframes = rpp.CoordinateFramesMarker(topic="rviz_pyplot/marker_array", defaultSize=0.1)

# This will use the default colors and size 
cframes.addCoordinateFrames(frames)

# Add a big coordinate frame at identity
colors = np.array([[0.8,0.3,0.3,1.0],  # x-axis color (rgba)
                   [0.3,0.8,0.3,1.0],  # y-axis color (rgba)
                   [0.3,0.3,0.8,1.0]]) # z-axis color (rgba)

cframes.addCoordinateFrame(np.eye(4), colors, 1.0)

plotter.plot(cframes)
