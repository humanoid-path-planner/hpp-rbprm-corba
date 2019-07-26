from hpp.corbaserver.rbprm.anymal_abstract import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver import ProblemSolver
from pinocchio import Quaternion
import numpy as np
import time
import math
statusFilename = "/res/infos.log"


vInit = 0.05# initial / final velocity to fix the direction
vGoal = 0.01
vMax = 0.5# linear velocity bound for the root
aMax = 0.5# linear acceleration bound for the root
extraDof = 6
mu=0.3# coefficient of friction
# Creating an instance of the helper class, and loading the robot
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
# Define bounds for the root : bounding box of the scenario
rootBounds = [0.4,3.6, 0.4, 2., 0.4, 0.5]
rbprmBuilder.setJointBounds ("root_joint", rootBounds)

# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
rbprmBuilder.setFilter([])
for rom in rbprmBuilder.urdfNameRom :
    rbprmBuilder.setAffordanceFilter(rom, ['Support'])

# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-3.14,3.14,-0.1,0.1,-0.1,0.1])
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
ps.setParameter("DynamicPlanner/friction",mu)
# sample only configuration with null velocity and acceleration :
ps.setParameter("ConfigurationShooter/sampleExtraDOF",False)

# initialize the viewer :
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

# load the module to analyse the environnement and compute the possible contact surfaces
from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel ("hpp_environments", "multicontact/plateforme_not_flat", "planning", vf, reduceSizes=[0.05,0,0])
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

# Generate random init and goal position.
X_BOUNDS=[0.4,3.6]
Y_BOUNDS=[0.4,2.]
Z_VALUE = 0.465


import random
random.seed()

q_init[0:3] = [random.uniform(X_BOUNDS[0],X_BOUNDS[1]),random.uniform(Y_BOUNDS[0],Y_BOUNDS[1]),Z_VALUE]
q_goal=q_init[::]
for i in range(random.randint(0,1000)):
  random.uniform(0.,1.)
q_goal[0:3] = [random.uniform(X_BOUNDS[0],X_BOUNDS[1]),random.uniform(Y_BOUNDS[0],Y_BOUNDS[1]),Z_VALUE]



# compute the orientation such that q_init face q_goal :
# set final orientation to be along the circle : 
vx = np.matrix([1,0,0]).T
v_init = np.matrix([q_goal[0]-q_init[0],q_goal[1]-q_init[1],0]).T
quat = Quaternion.FromTwoVectors(vx,v_init)
q_init[3:7] = quat.coeffs().T.tolist()[0]
q_goal[3:7] = q_init[3:7]



print "initial root position : ",q_init
print "final root position : ",q_goal
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
#ps.selectSteeringMethod("RBPRMKinodynamic")
#ps.selectDistance("Kinodynamic")
#ps.selectPathPlanner("DynamicPlanner")

# Solve the planning problem :
success = ps.client.problem.prepareSolveStepByStep()

if not success:
  print "planning failed."
  import sys
  sys.exit(1)

ps.client.problem.finishSolveStepByStep()


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

