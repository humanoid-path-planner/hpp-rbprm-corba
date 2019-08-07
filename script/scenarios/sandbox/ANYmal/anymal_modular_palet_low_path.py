from hpp.corbaserver.rbprm.anymal_abstract import Robot
Robot.urdfName += "_large"
from hpp.gepetto import Viewer
from hpp.corbaserver import ProblemSolver
from pinocchio import Quaternion
import numpy as np
import time
import math
statusFilename = "/res/infos.log"

Z_VALUE = 0.465
Y_VALUE = -0.1
vInit = 0.3
vMax = 0.5# linear velocity bound for the root
aMax = 0.5# linear acceleration bound for the root
aMaxZ = 5.
extraDof = 6
mu=0.3# coefficient of friction
# Creating an instance of the helper class, and loading the robot
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
# Define bounds for the root : bounding box of the scenario
rootBounds = [-1.7,1.7, Y_VALUE-0.001, Y_VALUE+0.001, Z_VALUE-0.001, Z_VALUE+0.231]
rbprmBuilder.setJointBounds ("root_joint", rootBounds)

# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
rbprmBuilder.setFilter(Robot.urdfNameRom)
for rom in rbprmBuilder.urdfNameRom :
    rbprmBuilder.setAffordanceFilter(rom, ['Support'])

# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-3.14,3.14,-0.1,0.1,-0.1,0.1])
# Add 6 extraDOF to the problem, used to store the linear velocity and acceleration of the root
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)
# We set the bounds of this extraDof with velocity and acceleration bounds (expect on z axis)
rbprmBuilder.client.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,-vMax,vMax,-aMax,aMax,-aMax,aMax,-aMaxZ,aMaxZ])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.robot.getDimensionExtraConfigSpace()

# Creating an instance of HPP problem solver 
ps = ProblemSolver( rbprmBuilder )
# define parameters used by various methods : 
ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
ps.setParameter("Kinodynamic/verticalAccelerationBound",aMaxZ)
ps.setParameter("Kinodynamic/forceAllOrientation",True)
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
afftool.loadObstacleModel ("hpp_environments", "ori/modular_palet_low", "planning", vf, reduceSizes=[0.06,0,0])
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

#v.addLandmark(v.sceneName,1)
q_init = rbprmBuilder.getCurrentConfig ();


q_init[2] = Z_VALUE
q_init[0:2] = [-1.6,Y_VALUE]
q_init[-6] = vInit
v(q_init)

q_goal=q_init[::]
q_goal[0] = 1.6


ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# write problem in files : 
f = open(statusFilename,"w")
f.write("q_init= "+str(q_init)+"\n")
f.write("q_goal= "+str(q_goal)+"\n")
f.close()


# Choosing RBPRM shooter and path validation methods.
ps.selectConfigurationShooter("RbprmShooter")
ps.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods :
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("Kinodynamic")
ps.selectPathPlanner("DynamicPlanner")
ps.addPathOptimizer ("RandomShortcutDynamic")


# Solve the planning problem :
t = ps.solve()
print "Guide planning done in "+str(t)+", optimizing trajectory ..."
pid = ps.numberPaths()-1

for i in range(20):
  ps.optimizePath(pid)
  pid = ps.numberPaths()-1

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

