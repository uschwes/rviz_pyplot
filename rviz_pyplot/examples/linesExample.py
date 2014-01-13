import roslib
roslib.load_manifest('rviz_pyplot')
import rviz_pyplot as rpp
reload(rpp)
import numpy as np

import transformations as tr

# Create N lines
N = 15


# Create a plotter
plotter = rpp.Plotter()

lines = rpp.LinesMarker(topic="rviz_pyplot/marker_array", lineWidth=0.01)



for i in range(0,N):
    # Generate a random line
    mz = -np.random.rand()*3.0
    xz = -np.random.rand()*3.0
    mt = -np.random.rand()*4.0
    xt = -np.random.rand()*4.0
    
    theta = np.linspace(-mt * np.pi, xt * np.pi, 100)
    z = np.linspace(-mz, xz, 100)
    r = z**2 + 1
    x = r * np.sin(theta)
    y = r * np.cos(theta)

    P = np.vstack([x,y,z]).T

    # Generate a random color
    color = np.random.rand(4)
    # Set the color alpha to 1
    color[3] = 1.0
    # The color is optional
    lines.addLineStrip(P, color)

# Plot the lines
plotter.plot(lines)
