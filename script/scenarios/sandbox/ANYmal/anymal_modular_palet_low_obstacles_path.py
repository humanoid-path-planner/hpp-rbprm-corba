from hpp.corbaserver.rbprm.anymal_abstract import Robot
Robot.urdfName += "_large"
from hpp.gepetto import Viewer
from hpp.corbaserver import ProblemSolver
from pinocchio import Quaternion
import numpy as np
import time
import math
statusFilename = "infos.log"
REF_Z_VALUE = 0.47
Z_VALUE_LOGS = REF_Z_VALUE + 0.13
Z_VALUE_PALET = REF_Z_VALUE + 0.165
Z_VALUE_WOOD = REF_Z_VALUE + 0.15
Y_VALUE = -0.13
vInit = 0.2
vMax = 0.5# linear velocity bound for the root
aMax = 0.8# linear acceleration bound for the root
aMaxZ = 5.
extraDof = 6
mu=0.5# coefficient of friction
# Creating an instance of the helper class, and loading the robot
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
# Define bounds for the root : bounding box of the scenario
rootBounds = [-1.201,1.11, Y_VALUE-0.3, Y_VALUE+0.3, Z_VALUE_LOGS - 0.001, Z_VALUE_PALET + 0.06]
rbprmBuilder.setJointBounds ("root_joint", rootBounds)

# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
rbprmBuilder.setFilter(Robot.urdfNameRom)
for rom in rbprmBuilder.urdfNameRom :
    rbprmBuilder.setAffordanceFilter(rom, ['Support'])

# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-0.3,0.3,-0.01,0.01,-0.01,0.01])
# Add 6 extraDOF to the problem, used to store the linear velocity and acceleration of the root
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)
# We set the bounds of this extraDof with velocity and acceleration bounds (expect on z axis)
rbprmBuilder.client.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,-2.,2.,-aMax,aMax,-aMax,aMax,-aMaxZ,aMaxZ])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.robot.getDimensionExtraConfigSpace()

# Creating an instance of HPP problem solver 
ps = ProblemSolver( rbprmBuilder )
# define parameters used by various methods : 
ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
ps.setParameter("Kinodynamic/verticalAccelerationBound",aMaxZ)
#ps.setParameter("Kinodynamic/synchronizeVerticalAxis",False)
ps.setParameter("Kinodynamic/forceYawOrientation",True)
ps.setParameter("DynamicPlanner/sizeFootX",0.01)
ps.setParameter("DynamicPlanner/sizeFootY",0.01)
ps.setParameter("DynamicPlanner/friction",mu)
# sample only configuration with null velocity and acceleration :
ps.setParameter("ConfigurationShooter/sampleExtraDOF",False)
ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops",500)

# initialize the viewer :
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

# load the module to analyse the environnement and compute the possible contact surfaces
from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.setMinimumArea('Support',0.03)
afftool.loadObstacleModel ("hpp_environments", "ori/modular_palet_low_obstacles", "planning", vf,reduceSizes=[0.1,0,0])
#afftool.loadObstacleModel ("hpp_environments", "ori/modular_palet_low", "planning", vf)

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




# Choosing RBPRM shooter and path validation methods.
ps.selectConfigurationShooter("RbprmShooter")
ps.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods :
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("Kinodynamic")
ps.selectPathPlanner("DynamicPlanner")
ps.addPathOptimizer ("RandomShortcutDynamic")
#ps.addPathOptimizer ("RandomShortcut")

# init to beginning of palet
q_init = rbprmBuilder.getCurrentConfig ();
q_init[2] = Z_VALUE_LOGS
q_init[0:2] = [-1.2,Y_VALUE]
q_init[-6] = vInit
v(q_init)
q_goal = q_init[::]
q_goal[0] = -0.55
q_goal[2] = Z_VALUE_PALET + 0.03
q_goal[3:7] = [ 0, -0.0499792, 0, 0.9987503 ]
q_goal[-6] = vInit
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# Solve the planning problem :
t = ps.solve()
print "Guide planning done in "+str(t)

pidBegin = ps.numberPaths()-1

## middle part on palet
q_init = q_goal[::]
q_goal[3:7] = [ 0, 0.0499792, 0, 0.9987503 ]
q_goal[0] = 0.55
q_goal[1] = -Y_VALUE
q_goal[-6] = vInit
ps.resetGoalConfigs()
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
rbprmBuilder.setJointBounds ("root_joint", [q_init[0],q_goal[0],-0.2,0.2,q_init[2],q_init[2]])
# Solve the planning problem :
t = ps.solve()
print "Guide planning done in "+str(t)
#v.solveAndDisplay('rm',2,0.001)
pidMiddle = ps.numberPaths()-1
rbprmBuilder.setJointBounds ("root_joint", rootBounds)

## last part on wood floor
q_init = q_goal[::]
q_goal[1] = Y_VALUE
q_goal[3:7] = [ 0, 0, 0, 1. ]
q_goal[0] = 1.1
q_goal[2] = Z_VALUE_WOOD
q_goal[-6] = vInit
ps.resetGoalConfigs()
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# Solve the planning problem :
t = ps.solve()
print "Guide planning done in "+str(t)

pidLast = ps.numberPaths()-1

# concatenate paths : 
pid = pidBegin
ps.concatenatePath(pid,pidMiddle)
ps.concatenatePath(pid,pidLast)

"""
ps.optimizePath(pid)
pid = ps.numberPaths()-1
"""
try :
    # display solution : 
    from hpp.gepetto import PathPlayer
    pp = PathPlayer (v)
    pp.dt=0.1
    pp.displayPath(pid)#pp.displayVelocityPath(0) #
    v.client.gui.setVisibility("path_"+str(pid)+"_root","ALWAYS_ON_TOP")
    pp.dt=0.01
    pp(pid)
except Exception:
    pass

# move the robot out of the view before computing the contacts
q_far = q_init[::]
q_far[2] = -2
v(q_far)

