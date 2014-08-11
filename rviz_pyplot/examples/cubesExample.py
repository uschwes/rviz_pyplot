import roslib
roslib.load_manifest('rviz_pyplot')
import rviz_pyplot as rpp
reload(rpp)
import numpy as np

import transformations as tr

# Create N cubes
N = 10

# Create a plotter
plotter = rpp.Plotter()

cubes = []

for i in range(0,N):
  
    # Generate a random cube
    scale = np.random.rand(3,1)*1.0
    p = np.random.rand(3,1)*5.0
    euler = np.random.rand(3,1)
    
    euler[1] = 0.0
    euler[2] = 0.0
    print "yaw =",euler[0]
    
    quat = tr.quaternion_from_euler(euler[0], euler[1], euler[2])
    
    pose=(p,quat)
    
    # Generate a random color
    color = np.random.rand(4)
    color[3] = 0.5 # the color alpha
    
    # The color is optional
    cubeMarker = rpp.CubeMarker(pose, scale=scale, color=color, markerId=i)   
    
    cubes.append(cubeMarker)

# Plot the lines
plotter.plot(cubes)
