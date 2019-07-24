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
mu=0.3# coefficient of friction
# Creating an instance of the helper class, and loading the robot
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
# Define bounds for the root : bounding box of the scenario
rootBounds = [0.1,4., 0.2, 2.2, 0.95, 1.05]
rbprmBuilder.setJointBounds ("root_joint", rootBounds)

# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
rbprmBuilder.setFilter([])#'talos_lleg_rom','talos_rleg_rom'])
rbprmBuilder.setAffordanceFilter('talos_lleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('talos_rleg_rom', ['Support'])
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
afftool.loadObstacleModel ("hpp_environments", "multicontact/plateforme_not_flat", "planning", vf, reduceSizes=[0.1,0,0])
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

# Generate random init and goal position.
# these position will be on the flat part of the environment, with an orientation such that they can be connected by a straight line, and an angle between +- 25 degree from the x axis
Y_BOUNDS=[0.3,2.1]
Z_VALUE = 0.98
MAX_ANGLE = 0.4363323129985824 # 25 degree

import random
random.seed()

# select randomly the initial and final plateform, they cannot be the same
# end plateform is always after the init plateform on the x axis
init_plateform_id = random.randint(0,3)
end_plateform_id = random.randint(init_plateform_id+1,4)
#if end_plateform_id >= init_plateform_id:
#  end_plateform_id+=1

# set x position from the plateform choosen : 
x_init = 0.16 + 0.925*init_plateform_id
x_goal = 0.16 + 0.925*end_plateform_id

# uniformly sample y position
y_init = random.uniform(Y_BOUNDS[0],Y_BOUNDS[1])
q_init[0:3] = [x_init,y_init, Z_VALUE]

# y_goal must be random inside Y_BOUNDS but such that the line between q_init and q_goal is between +- MAX_ANGLE radian from the x axis 
y_bound_goal = Y_BOUNDS[::]
y_angle_max = math.sin(MAX_ANGLE)*abs(x_init-x_goal) + y_init
y_angle_min = math.sin(-MAX_ANGLE)*abs(x_init-x_goal) + y_init
y_bound_goal[0] = max(y_angle_min,y_bound_goal[0])
y_bound_goal[1] = min(y_angle_max,y_bound_goal[1])
y_goal = random.uniform(y_bound_goal[0],y_bound_goal[1])



# compute the orientation such that q_init face q_goal :
# set final orientation to be along the circle : 
vx = np.matrix([1,0,0]).T
v_init = np.matrix([x_goal-x_init,y_goal-y_init,0]).T
quat = Quaternion.FromTwoVectors(vx,v_init)
q_init[3:7] = quat.coeffs().T.tolist()[0]

q_goal=q_init[::]
q_goal[0:2] = [x_goal,y_goal]

print "initial root position : ",q_init
print "final root position : ",q_goal
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

