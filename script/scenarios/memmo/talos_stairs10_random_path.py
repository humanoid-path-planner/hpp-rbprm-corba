from hpp.corbaserver.rbprm.talos_abstract import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import time
statusFilename = "infos.log"
Robot.urdfName+="_large"

vMax = 0.3# linear velocity bound for the root
aMax = 0.05 # linear acceleration bound for the root
extraDof = 6
mu=0.5# coefficient of friction
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot()
# Define bounds for the root : bounding box of the scenario
root_bounds = [-0.9,2.55,-0.13,2., 0.98, 1.6]
rbprmBuilder.setJointBounds ("root_joint", root_bounds)
# As this scenario only consider walking, we fix the DOF of the torso :
rbprmBuilder.setJointBounds ('torso_1_joint', [0,0])
rbprmBuilder.setJointBounds ('torso_2_joint', [0.,0.])

# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
rbprmBuilder.setFilter(['talos_lleg_rom','talos_rleg_rom'])
rbprmBuilder.setAffordanceFilter('talos_lleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('talos_rleg_rom', ['Support'])
# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-4.,4.,-0.1,0.1,-0.1,0.1])
# Add 6 extraDOF to the problem, used to store the linear velocity and acceleration of the root
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)
# We set the bounds of this extraDof with velocity and acceleration bounds (expect on z axis)
rbprmBuilder.client.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,-10.,10.,-aMax,aMax,-aMax,aMax,-10.,10.])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.robot.getDimensionExtraConfigSpace()

# Creating an instance of HPP problem solver
ps = ProblemSolver( rbprmBuilder )
# define parameters used by various methods : 
ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
# force the orientation of the trunk to match the direction of the motion
ps.setParameter("Kinodynamic/forceYawOrientation",True)
ps.setParameter("DynamicPlanner/sizeFootX",0.2)
ps.setParameter("DynamicPlanner/sizeFootY",0.12)
ps.setParameter("DynamicPlanner/friction",mu)
# sample only configuration with null velocity and acceleration :
ps.setParameter("ConfigurationShooter/sampleExtraDOF",False)
ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops",100)

# initialize the viewer :
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

# load the module to analyse the environnement and compute the possible contact surfaces
from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel ("hpp_environments", "multicontact/bauzil_stairs", "planning", vf,reduceSizes=[0.18,0.,0.])
#load the viewer
try :
    v = vf.createViewer(displayArrows = True)
except Exception:
    print "No viewer started !"
    class FakeViewer():
        sceneName = ""
        def __init__(self):
            return
        def __call__(self,q):
            return
        def addLandmark(self,a,b):
            return
    v = FakeViewer()
#afftool.visualiseAffordances('Support', v, v.color.lightBrown)
v.addLandmark(v.sceneName,1)

#####
import random
import numpy as np
from pinocchio import Quaternion
from tools.sampleRotation import sampleRotationForConfig
"""
# rectangle used to sample position of the floor
x_floor=[-0.9,0.05]
y_floor=[0.6,2.]
z_floor=0.98
# rectangle used to sample position on the plateform
x_plat=[1.9,2.55]
y_plat=[-0.13,1.53]
z_plat=1.58
"""
# rectangle used to sample position of the floor
x_floor=[-0.9,0.05]
y_floor=[0.8,1.50]
z_floor=0.98
# rectangle used to sample position on the plateform
x_plat=[1.9,2.55]
y_plat=[0.8,1.50]
z_plat=1.58
vPredef = 0.05
vx = np.matrix([1,0,0]).T
#####


q_up = rbprmBuilder.getCurrentConfig ()
q_down = q_up[::]
#generate a random problem : (q_init, q_goal)
random.seed()
go_up = random.randint(0,1)
if go_up:
    print "go upstair"
    alphaBounds=[np.pi/4., 3.*np.pi/4.]
else :
    print "go downstair"
    alphaBounds=[5.*np.pi/4., 7.*np.pi/4.]

# sample random valid position on the floor : 
while not rbprmBuilder.isConfigValid(q_down)[0]:
    q_down[0] = random.uniform(x_floor[0],x_floor[1])
    q_down[1] = random.uniform(y_floor[0],y_floor[1])
    q_down[2] = z_floor
    # sample random orientation : 
    q_down = sampleRotationForConfig(alphaBounds,q_down,vPredef)
# sample random valid position on the platform : 
while not rbprmBuilder.isConfigValid(q_up)[0]:
    q_up[0] = random.uniform(x_plat[0],x_plat[1])
    q_up[1] = random.uniform(y_plat[0],y_plat[1])
    q_up[2] = z_plat
    # sample random orientation : 
    q_up = sampleRotationForConfig(alphaBounds,q_up,vPredef)


if go_up:
    q_init = q_down
    q_goal = q_up
else :
    q_init = q_up
    q_goal = q_down

# write problem in files : 
f = open(statusFilename,"w")
f.write("q_init= "+str(q_init)+"\n")
f.write("q_goal= "+str(q_goal)+"\n")
f.close()


ps.setInitialConfig (q_init)
v(q_init)
ps.addGoalConfig (q_goal)
v(q_goal)

# Choosing RBPRM shooter and path validation methods.
ps.selectConfigurationShooter("RbprmShooter")
ps.addPathOptimizer ("RandomShortcutDynamic")
ps.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods :
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("Kinodynamic")
ps.selectPathPlanner("DynamicPlanner")

# Solve the planning problem :

t = ps.solve ()
print "Guide planning time : ",t
#pId = ps.numberPaths()-1
pId = 0

# display solution :
try : 
    from hpp.gepetto import PathPlayer
    pp = PathPlayer (v)
    pp.dt=0.1
    pp.displayVelocityPath(pId)
    v.client.gui.setVisibility("path_0_root","ALWAYS_ON_TOP")
    pp.dt = 0.01
    pp(pId)
except Exception:
    pass
# move the robot out of the view before computing the contacts
q_far = q_init[::]
q_far[2] = -2
v(q_far)

