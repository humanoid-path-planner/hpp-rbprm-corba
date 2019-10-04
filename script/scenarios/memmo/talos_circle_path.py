from hpp.corbaserver.rbprm.talos_abstract import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver import ProblemSolver
import numpy as np
import time
statusFilename = "/res/infos.log"


vMax = 0.5# linear velocity bound for the root
aMax = 0.5# linear acceleration bound for the root
extraDof = 6
mu=0.5# coefficient of friction
# Creating an instance of the helper class, and loading the robot
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
# Define bounds for the root : bounding box of the scenario
rbprmBuilder.setJointBounds ("root_joint", [-2,2, -2, 2, 0.95, 1.05])

# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
rbprmBuilder.setFilter(['talos_lleg_rom','talos_rleg_rom'])
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
afftool.loadObstacleModel ("hpp_environments", "multicontact/ground", "planning", vf)
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

q_init = rbprmBuilder.getCurrentConfig ();
q_init[0:3] = [0,0,1.]
q_init[3:7] = [0,0,0,1]

# sample random position on a circle of radius 2m

radius = 0.3
import random 
random.seed()
alpha = random.uniform(0.,2.*np.pi)
print "Test on a circle, alpha = ",alpha
q_goal = q_init[::]
q_goal [0:3] = [radius*np.sin(alpha), -radius*np.cos(alpha), 1.]

print "initial root position : ",q_init[0:3]
print "final root position : ",q_goal[0:3]
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

# Solve the planning problem :
t = ps.solve ()
print "Guide planning time : ",t

try :
    # display solution : 
    from hpp.gepetto import PathPlayer
    pp = PathPlayer (v)
    pp.dt=0.1
    pp.displayVelocityPath(0)
    #v.client.gui.setVisibility("path_0_root","ALWAYS_ON_TOP")
    pp.dt=0.01
    #pp(0)
except Exception:
    pass

# move the robot out of the view before computing the contacts
q_far = q_init[::]
q_far[2] = -2
v(q_far)

