from hpp.corbaserver.rbprm.talos_abstract import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver import ProblemSolver
from pinocchio import Quaternion
import numpy as np
import time
import math
statusFilename = "infos.log"

vInit = 0.05# initial / final velocity to fix the direction
vGoal = 0.01
vMax = 0.5# linear velocity bound for the root
aMax = 0.5# linear acceleration bound for the root
extraDof = 6
mu=0.5# coefficient of friction
# Creating an instance of the helper class, and loading the robot
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
# Define bounds for the root : bounding box of the scenario
rootBounds = [0.2,3.9, 0.2, 2.2, 0.95, 1.05]
rbprmBuilder.setJointBounds ("root_joint", rootBounds)

# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
rbprmBuilder.setFilter([])#'talos_lleg_rom','talos_rleg_rom'])
rbprmBuilder.setAffordanceFilter('talos_lleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('talos_rleg_rom', ['Support'])
# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-1.7,1.7,-0.1,0.1,-0.1,0.1])
# Add 6 extraDOF to the problem, used to store the linear velocity and acceleration of the root
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)
# We set the bounds of this extraDof with velocity and acceleration bounds (expect on z axis)
rbprmBuilder.client.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,0,0,-aMax,aMax,-aMax,aMax,0,0])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.robot.getDimensionExtraConfigSpace()

# Creating an instance of HPP problem solver 
ps = ProblemSolver( rbprmBuilder )
# define parameters used by various methods : 
ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
ps.setParameter("DynamicPlanner/sizeFootX",0.2)
ps.setParameter("DynamicPlanner/sizeFootY",0.12)
ps.setParameter("DynamicPlanner/friction",0.5)
# sample only configuration with null velocity and acceleration :
ps.setParameter("ConfigurationShooter/sampleExtraDOF",False)

# initialize the viewer :
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

# load the module to analyse the environnement and compute the possible contact surfaces
from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel ("hpp_environments", "multicontact/plateforme_not_flat", "planning", vf, reduceSizes=[0.15,0,0])
try :
    v = vf.createViewer(displayArrows = True)
except Exception:
    print "No viewer started !"
    class FakeViewer():
        def __init__(self):
            return
        def __call__(self,q):
            return
    v = FakeViewer()
    
#afftool.visualiseAffordances('Support', v, v.color.lightBrown)

v.addLandmark(v.sceneName,1)
q_init = rbprmBuilder.getCurrentConfig ();
q_init[0:3] = [0.20,1.15,0.99]
#q_init[0:3] = [0.20,1.15,1.0]
q_init[3:7] = [0,0,0,1]
print "q_init",q_init[1]

# Q init => Set position and orientation
# X will be between [groundMinX, groundMaxX]
# Y will be between [groundMinY, groundMaxY]
lengthPath = 0.9
marginX = 1.7
marginY = 0.8
groundMinX = marginX
groundMaxX = 4.0 - marginX
groundMinY = marginY
groundMaxY = 2.4 - marginY

minAngleDegree = 80
maxAngleDegree = 100

positionIsRandomOnFlatGround = True

# INIT
radius = 0.01
import random
random.seed()
alpha = 0.0
if random.uniform(0.,1.) > 0.5:
	alpha = random.uniform(minAngleDegree,maxAngleDegree)
else:
	alpha = random.uniform(minAngleDegree+180,maxAngleDegree+180)
print "Test on a circle, alpha deg = ",alpha
alpha = alpha*np.pi/180.0
move_Y = random.uniform(-0.2,0.2)

q_init_random= q_init[::]
q_init_random [0:3] = [radius*np.sin(alpha), -radius*np.cos(alpha), 1.]
# set final orientation to be along the circle : 
vx = np.matrix([1,0,0]).T
v_init = np.matrix([q_init_random[0],q_init_random[1],0]).T
quat = Quaternion.FromTwoVectors(vx,v_init)
q_init_random[3:7] = quat.coeffs().T.tolist()[0]
# set initial velocity to fix the orientation
q_init_random[-6] = vInit*np.sin(alpha)
q_init_random[-5] = -vInit*np.cos(alpha)
if positionIsRandomOnFlatGround :
	# Set robot on flat ground
	q_init_random[0] = 2.0
	q_init_random[1] = 1.2 + move_Y
else:
	# Set robot at random position on platform
	q_init_random[0] = random.uniform(groundMinX,groundMaxX)
	q_init_random[1] = random.uniform(groundMinY,groundMaxY)
v(q_init_random)

# GOAL
# Q goal => Set straight position in square of size (4,2)
# 	    Orientation is the vector between position init and goal 
q_goal_random = q_init_random[::]
# Set robot goal at random position on platform
q_goal_random[0] = q_init_random[0] + np.sin(alpha)*lengthPath
q_goal_random[1] = q_init_random[1] - np.cos(alpha)*lengthPath
v(q_goal_random)
# Set new q_init and q_goal
q_init = q_init_random[::]
q_goal = q_goal_random[::]

print "initial root position/velocity : ",q_init
print "final root position/velocity : ",q_goal
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# write problem in files : 
"""
f = open(statusFilename,"w")
f.write("q_init= "+str(q_init)+"\n")
f.write("q_goal= "+str(q_goal)+"\n")
f.close()
"""

# Choosing RBPRM shooter and path validation methods.
ps.selectConfigurationShooter("RbprmShooter")
ps.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods :
#ps.selectSteeringMethod("RBPRMKinodynamic")
#ps.selectDistance("Kinodynamic")
#ps.selectPathPlanner("DynamicPlanner")

# Solve the planning problem :
t = ps.solve ()
print "Guide planning time : ",t

try :
    # display solution : 
    from hpp.gepetto import PathPlayer
    pp = PathPlayer (v)
    pp.dt=0.1
    pp.displayPath(0)#pp.displayVelocityPath(0) #
    #v.client.gui.setVisibility("path_0_root","ALWAYS_ON_TOP")
    pp.dt=0.01
    #pp(0)
except Exception:
    pass

# move the robot out of the view before computing the contacts
q_far = q_init[::]
q_far[2] = -2
v(q_far)

