import roslib
roslib.load_manifest('rviz_plotting')
import rviz_plotting as rvp
reload(rvp)
import numpy as np;
roslib.load_manifest('aslam_python') 
import aslam
roslib.load_manifest('sm_python')
import sm
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point


T_km1_k = sm.Transformation(np.array([0,0,0,1.0]), np.array([0.1,0,0]))
# DiscreteTrajectory createTrajectoryRandomWalk( NsecTime t0, Transformation T0, size_t numSteps, Transformation deterministicStep, Vector3d translationStdDev, Vector3d  yawPitchRollStdDev, NsecTime dt
traj = aslam.simulation.createTrajectoryRandomWalk(0, sm.Transformation(), 100, T_km1_k, np.array([0.01,0.01,0.01]), np.array([0.01,0,0]), 1000000)

TT = [ traj.T(long(t)).T() for t in traj.getSupportTimes() ] 

plotter = rvp.Plotter()
plotter.plotTrajectory( TT, markerId=2 )

N = 1000
S = 10.0
Y = S * np.random.rand(N,3)

namedChannels = []
namedChannels.append( ('height', np.asarray(Y[:,2], dtype=np.int8) ) )
namedChannels.append( ('blah', np.asarray(Y[:,1], dtype=np.int32) ) )

plotter.plot3(Y)#,namedChannels=namedChannels)
plotter.plot3(Y,namedChannels=namedChannels)
